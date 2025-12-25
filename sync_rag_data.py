#!/usr/bin/env python3
"""
RAG Data Sync Script

This script:
1. Deletes the existing Qdrant collection (all vector data)
2. Clears old RAG data from Neon PostgreSQL
3. Re-indexes documents from ./docs folder into Qdrant with proper paths/references
4. Auto-pushes changes to git after completion

Usage:
    python sync_rag_data.py

Environment variables required (in backend/.env):
    - QDRANT_URL
    - QDRANT_API_KEY
    - QDRANT_COLLECTION_NAME
    - NEON_DATABASE_URL
    - OPENAI_API_KEY
"""

import os
import sys
import subprocess
from datetime import datetime

# Add backend to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

from dotenv import load_dotenv
load_dotenv(os.path.join(os.path.dirname(__file__), 'backend', '.env'))

from qdrant_client import QdrantClient
from qdrant_client.http.models import Distance, VectorParams, PointStruct
from sqlalchemy import text
from sqlalchemy.engine import create_engine
from langchain_community.document_loaders import DirectoryLoader, TextLoader
from langchain_text_splitters import RecursiveCharacterTextSplitter
from langchain_huggingface import HuggingFaceEmbeddings
import hashlib
import json

# Configuration
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "robotics_docs")
NEON_DATABASE_URL = os.getenv("NEON_DATABASE_URL")

# Base URL for documentation links
DOCS_BASE_URL = "https://muhammadariyan.github.io/Ary-s-Physical-Humanoid-Robotics/docs"


def get_doc_url_path(file_path: str) -> str:
    """Convert file path to documentation URL path"""
    # Remove ./docs/ prefix and .md extension
    path = file_path.replace("./docs/", "").replace("docs/", "")
    path = path.replace(".md", "")

    # Convert to URL format (remove numbers prefix like 01-, 02a-, etc.)
    parts = path.split("/")
    clean_parts = []
    for part in parts:
        # Remove numeric prefixes like "01-", "02a-"
        if part and len(part) > 2:
            import re
            clean_part = re.sub(r'^\d+[a-z]?-', '', part)
            clean_parts.append(clean_part)
        else:
            clean_parts.append(part)

    return "/".join(clean_parts)


def delete_qdrant_collection():
    """Delete the existing Qdrant collection"""
    print(f"\n{'='*60}")
    print(f"STEP 1: Deleting Qdrant collection: {QDRANT_COLLECTION_NAME}")
    print(f"{'='*60}")

    try:
        if QDRANT_API_KEY:
            client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
        else:
            client = QdrantClient(url=QDRANT_URL)

        try:
            client.get_collection(QDRANT_COLLECTION_NAME)
            client.delete_collection(QDRANT_COLLECTION_NAME)
            print(f"   [OK] Collection '{QDRANT_COLLECTION_NAME}' deleted")
        except Exception as e:
            if "Forbidden" in str(e) or "403" in str(e):
                print(f"   [ERROR] Permission denied! Your API key is READ-ONLY.")
                print(f"   [ERROR] To run this script, you need a GLOBAL ACCESS key.")
                print(f"   [ERROR] Get it from Qdrant Cloud dashboard -> API Keys -> Create with 'Manage' access")
                sys.exit(1)
            print(f"   [OK] Collection '{QDRANT_COLLECTION_NAME}' does not exist (skipping)")

    except Exception as e:
        if "Forbidden" in str(e) or "403" in str(e):
            print(f"   [ERROR] Permission denied! Your API key is READ-ONLY.")
            print(f"   [ERROR] To run this script, you need a GLOBAL ACCESS key.")
            print(f"   [ERROR] Get it from Qdrant Cloud dashboard -> API Keys -> Create with 'Manage' access")
            sys.exit(1)
        print(f"   [ERROR] Error deleting Qdrant collection: {e}")
        raise


def clear_neon_database():
    """Clear chat sessions and messages from Neon PostgreSQL"""
    print(f"\n{'='*60}")
    print(f"STEP 2: Clearing Neon database")
    print(f"{'='*60}")

    try:
        engine = create_engine(NEON_DATABASE_URL)

        with engine.connect() as conn:
            # Clear chat_messages first (foreign key constraint)
            result = conn.execute(text("DELETE FROM chat_messages"))
            print(f"   [OK] Deleted {result.rowcount} chat_messages")

            # Clear chat_sessions
            result = conn.execute(text("DELETE FROM chat_sessions"))
            print(f"   [OK] Deleted {result.rowcount} chat_sessions")

            conn.commit()

    except Exception as e:
        print(f"   [ERROR] Error clearing Neon database: {e}")
        raise


def index_docs_folder():
    """Index documents from ./docs folder into Qdrant with proper paths"""
    print(f"\n{'='*60}")
    print(f"STEP 3: Indexing documents from ./docs folder")
    print(f"{'='*60}")

    docs_path = "./docs"

    if not os.path.exists(docs_path):
        print(f"   [ERROR] Docs folder not found: {docs_path}")
        return 0

    # Load documents
    loader = DirectoryLoader(
        docs_path,
        glob="**/*.md",
        loader_cls=TextLoader,
        show_progress=True
    )

    documents = loader.load()
    print(f"   [OK] Loaded {len(documents)} documents")

    if not documents:
        print("   [WARN] No documents found to index")
        return 0

    # Split documents into chunks
    text_splitter = RecursiveCharacterTextSplitter(
        chunk_size=1000,
        chunk_overlap=200,
        separators=["\n## ", "\n### ", "\n\n", "\n", ". ", ", ", " "]
    )

    splits = text_splitter.split_documents(documents)
    print(f"   [OK] Split into {len(splits)} chunks")

    # Create HuggingFace embeddings (FREE, no API key needed)
    # Using BAAI/bge-large-en-v1.5 - 1024 dimensions, high quality
    print("   [OK] Initializing HuggingFace embeddings (BAAI/bge-large-en-v1.5)...")
    embeddings = HuggingFaceEmbeddings(
        model_name="BAAI/bge-large-en-v1.5",
        model_kwargs={'device': 'cpu'},
        encode_kwargs={'normalize_embeddings': True}
    )

    # Connect to Qdrant
    if QDRANT_API_KEY:
        client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
    else:
        client = QdrantClient(url=QDRANT_URL)

    # Create collection
    print(f"   [OK] Creating collection: {QDRANT_COLLECTION_NAME}")

    try:
        client.create_collection(
            collection_name=QDRANT_COLLECTION_NAME,
            vectors_config=VectorParams(
                size=1024,  # BAAI/bge-large-en-v1.5 dimension
                distance=Distance.COSINE
            )
        )
    except Exception as e:
        if "Forbidden" in str(e) or "403" in str(e):
            print(f"\n   [ERROR] Permission denied! Your API key is READ-ONLY.")
            print(f"   [ERROR] To run this script, you need a GLOBAL ACCESS key.")
            print(f"   [ERROR] Get it from Qdrant Cloud dashboard -> API Keys -> Create with 'Manage' access")
            sys.exit(1)
        raise

    # Process and upload documents with enhanced metadata
    print(f"   [OK] Processing and uploading {len(splits)} document chunks...")

    points = []
    for i, doc in enumerate(splits):
        # Get source path
        source_path = doc.metadata.get('source', '')

        # Generate URL path
        url_path = get_doc_url_path(source_path)
        full_url = f"{DOCS_BASE_URL}/{url_path}"

        # Extract section from content (first heading if exists)
        content = doc.page_content
        section = ""
        for line in content.split("\n"):
            if line.startswith("#"):
                section = line.lstrip("#").strip()
                break

        # Get filename without extension
        filename = os.path.basename(source_path).replace(".md", "")

        # Get parent folder (part name)
        parent_folder = os.path.dirname(source_path).split("/")[-1]

        # Create embedding
        embedding = embeddings.embed_query(content)

        # Enhanced metadata
        metadata = {
            "source": source_path,
            "title": filename,
            "section": section,
            "url": full_url,
            "url_path": url_path,
            "part": parent_folder,
            "chunk_index": i,
            "content_preview": content[:200] + "..." if len(content) > 200 else content,
            "indexed_at": datetime.utcnow().isoformat(),
        }

        # Create point
        point = PointStruct(
            id=i,
            vector=embedding,
            payload={
                "page_content": content,
                "metadata": metadata
            }
        )
        points.append(point)

        if (i + 1) % 50 == 0:
            print(f"      Processed {i + 1}/{len(splits)} chunks...")

    # Upload points in batches
    batch_size = 100
    for i in range(0, len(points), batch_size):
        batch = points[i:i + batch_size]
        client.upsert(
            collection_name=QDRANT_COLLECTION_NAME,
            points=batch
        )

    print(f"   [OK] Successfully indexed {len(splits)} document chunks")

    # Print summary of indexed documents
    print(f"\n   Indexed documents summary:")
    indexed_files = set()
    for doc in splits:
        indexed_files.add(doc.metadata.get('source', ''))

    for f in sorted(indexed_files):
        url_path = get_doc_url_path(f)
        print(f"      - {f}")
        print(f"        URL: {DOCS_BASE_URL}/{url_path}")

    return len(splits)


def git_push():
    """Auto-push changes to git"""
    print(f"\n{'='*60}")
    print(f"STEP 4: Auto-pushing to git")
    print(f"{'='*60}")

    try:
        # Check if there are any changes
        result = subprocess.run(
            ["git", "status", "--porcelain"],
            capture_output=True,
            text=True,
            cwd=os.path.dirname(__file__) or "."
        )

        if not result.stdout.strip():
            print("   [OK] No changes to commit")
            return

        # Add all changes
        subprocess.run(["git", "add", "-A"], check=True, cwd=os.path.dirname(__file__) or ".")
        print("   [OK] Staged all changes")

        # Commit
        commit_msg = f"chore: sync RAG data - {datetime.now().strftime('%Y-%m-%d %H:%M')}"
        subprocess.run(
            ["git", "commit", "-m", commit_msg],
            check=True,
            cwd=os.path.dirname(__file__) or "."
        )
        print(f"   [OK] Committed: {commit_msg}")

        # Push
        subprocess.run(
            ["git", "push", "origin", "main"],
            check=True,
            cwd=os.path.dirname(__file__) or "."
        )
        print("   [OK] Pushed to origin/main")

    except subprocess.CalledProcessError as e:
        print(f"   [WARN] Git operation failed: {e}")
    except Exception as e:
        print(f"   [ERROR] Git error: {e}")


def main():
    print("\n" + "=" * 60)
    print("  RAG DATA SYNC SCRIPT")
    print("  Syncs documentation to Qdrant with proper paths/references")
    print("  Using: BAAI/bge-large-en-v1.5 (1024 dims, FREE)")
    print("=" * 60)

    start_time = datetime.now()

    # Validate configuration
    print(f"\nConfiguration:")
    print(f"  QDRANT_URL: {QDRANT_URL}")
    print(f"  QDRANT_COLLECTION: {QDRANT_COLLECTION_NAME}")
    print(f"  NEON_DATABASE_URL: {'***configured***' if NEON_DATABASE_URL else 'NOT SET'}")
    print(f"  Embeddings: BAAI/bge-large-en-v1.5 (FREE, local)")

    if not all([QDRANT_URL, NEON_DATABASE_URL]):
        print("\n[ERROR] Missing required environment variables!")
        sys.exit(1)

    # Step 1: Delete Qdrant collection
    delete_qdrant_collection()

    # Step 2: Clear Neon database
    clear_neon_database()

    # Step 3: Index new documents
    chunks_indexed = index_docs_folder()

    # Step 4: Auto-push to git
    git_push()

    # Summary
    duration = datetime.now() - start_time
    print(f"\n{'='*60}")
    print(f"  SYNC COMPLETE!")
    print(f"{'='*60}")
    print(f"\nSummary:")
    print(f"  - Qdrant collection recreated: {QDRANT_COLLECTION_NAME}")
    print(f"  - Document chunks indexed: {chunks_indexed}")
    print(f"  - Duration: {duration.total_seconds():.1f} seconds")
    print(f"\nThe RAG system now has fresh data with proper paths!")
    print(f"Each document chunk includes:")
    print(f"  - source: original file path")
    print(f"  - url: full documentation URL")
    print(f"  - section: heading/section name")
    print(f"  - part: which part of the course")


if __name__ == "__main__":
    main()
