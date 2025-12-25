#!/usr/bin/env python3
"""
Cleanup and Reindex Script

This script:
1. Deletes the existing Qdrant collection (all vector data)
2. Clears all chat sessions and messages from Neon PostgreSQL
3. Re-indexes documents from ./docs folder into Qdrant

Usage:
    python cleanup_and_reindex.py
"""

import os
import sys

# Add backend to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

from dotenv import load_dotenv
load_dotenv(os.path.join(os.path.dirname(__file__), 'backend', '.env'))

from qdrant_client import QdrantClient
from sqlalchemy import text
from sqlalchemy.engine import create_engine
from langchain_community.document_loaders import DirectoryLoader, TextLoader
from langchain.text_splitter import RecursiveCharacterTextSplitter
from langchain_openai import OpenAIEmbeddings
from qdrant_client.http.models import Distance, VectorParams
import hashlib

# Configuration
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "robotics_docs")
NEON_DATABASE_URL = os.getenv("NEON_DATABASE_URL")
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")

def delete_qdrant_collection():
    """Delete the existing Qdrant collection"""
    print(f"\n🗑️  Deleting Qdrant collection: {QDRANT_COLLECTION_NAME}")

    try:
        if QDRANT_API_KEY:
            client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
        else:
            client = QdrantClient(url=QDRANT_URL)

        # Check if collection exists
        try:
            client.get_collection(QDRANT_COLLECTION_NAME)
            # Delete the collection
            client.delete_collection(QDRANT_COLLECTION_NAME)
            print(f"   ✓ Collection '{QDRANT_COLLECTION_NAME}' deleted successfully")
        except Exception:
            print(f"   ✓ Collection '{QDRANT_COLLECTION_NAME}' does not exist (skipping)")

    except Exception as e:
        print(f"   ✗ Error deleting Qdrant collection: {e}")
        raise

def clear_neon_database():
    """Clear all chat sessions and messages from Neon PostgreSQL"""
    print(f"\n🗑️  Clearing Neon database: chat_sessions and chat_messages")

    try:
        engine = create_engine(NEON_DATABASE_URL)

        with engine.connect() as conn:
            # Clear chat_messages first (foreign key constraint)
            result = conn.execute(text("DELETE FROM chat_messages"))
            print(f"   ✓ Deleted {result.rowcount} chat_messages")

            # Clear chat_sessions
            result = conn.execute(text("DELETE FROM chat_sessions"))
            print(f"   ✓ Deleted {result.rowcount} chat_sessions")

            conn.commit()

    except Exception as e:
        print(f"   ✗ Error clearing Neon database: {e}")
        raise

def index_docs_folder():
    """Index documents from ./docs folder into Qdrant"""
    print(f"\n📚 Indexing documents from ./docs folder...")

    docs_path = "./docs"

    if not os.path.exists(docs_path):
        print(f"   ✗ Docs folder not found: {docs_path}")
        return

    # Load documents from ./docs
    loader = DirectoryLoader(
        docs_path,
        glob="**/*",
        loader_cls=TextLoader,
        show_progress=True
    )

    documents = loader.load()
    print(f"   ✓ Loaded {len(documents)} documents")

    if not documents:
        print("   ⚠ No documents found to index")
        return

    # Split documents into chunks
    text_splitter = RecursiveCharacterTextSplitter(
        chunk_size=1000,
        chunk_overlap=200,
        separators=["\n\n", "\n", ". ", ", ", " "]
    )

    splits = text_splitter.split_documents(documents)
    print(f"   ✓ Split into {len(splits)} chunks")

    if not OPENAI_API_KEY:
        print("   ✗ OPENAI_API_KEY not set - cannot create embeddings")
        return

    # Create embeddings and upload to Qdrant
    embeddings = OpenAIEmbeddings(
        model="text-embedding-3-small",
        openai_api_key=OPENAI_API_KEY
    )

    # Connect to Qdrant
    if QDRANT_API_KEY:
        client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
    else:
        client = QdrantClient(url=QDRANT_URL)

    # Create collection with proper configuration
    print(f"   ✓ Creating collection: {QDRANT_COLLECTION_NAME}")

    client.create_collection(
        collection_name=QDRANT_COLLECTION_NAME,
        vectors_config=VectorParams(
            size=1536,  # text-embedding-3-small dimension
            distance=Distance.COSINE
        )
    )

    # Upload documents in batches
    from langchain_qdrant import Qdrant
    from qdrant_client.http.api_client import UnexpectedResponse

    qdrant_store = Qdrant(
        client=client,
        collection_name=QDRANT_COLLECTION_NAME,
        embeddings=embeddings,
    )

    print(f"   ✓ Uploading {len(splits)} document chunks to Qdrant...")

    # Upload with metadata
    from langchain.docstore.document import Document

    # Add source metadata to each document
    for doc in splits:
        # Extract source filename from path
        source_path = doc.metadata.get('source', '')
        filename = os.path.basename(source_path)
        doc.metadata['title'] = filename

    # Upload using add_documents
    qdrant_store.add_documents(splits)

    print(f"   ✓ Successfully indexed {len(splits)} document chunks")

    # Update document hash for future rebuild detection
    update_docs_hash(docs_path)

def update_docs_hash(docs_path):
    """Update the document hash file"""
    hash_md5 = hashlib.md5()

    if not os.path.exists(docs_path):
        return

    for root, dirs, files in os.walk(docs_path):
        for file in sorted(files):
            file_path = os.path.join(root, file)
            if os.path.isfile(file_path):
                hash_md5.update(file_path.encode('utf-8'))
                hash_md5.update(str(os.path.getmtime(file_path)).encode('utf-8'))

    import tempfile
    hash_file_path = os.path.join(tempfile.gettempdir(), f".{QDRANT_COLLECTION_NAME}_docs_hash")

    with open(hash_file_path, 'w') as f:
        f.write(hash_md5.hexdigest())

    print(f"   ✓ Updated document hash")

def main():
    print("=" * 60)
    print("🔄 Cleanup and Reindex Script")
    print("=" * 60)

    # Step 1: Delete Qdrant collection
    delete_qdrant_collection()

    # Step 2: Clear Neon database
    clear_neon_database()

    # Step 3: Index new documents
    index_docs_folder()

    print("\n" + "=" * 60)
    print("✅ Cleanup and reindex complete!")
    print("=" * 60)
    print("\nWhat was done:")
    print("  1. Deleted old Qdrant vector collection")
    print("  2. Cleared all chat data from Neon PostgreSQL")
    print("  3. Indexed new documents from ./docs folder")
    print("\nThe system is now ready with fresh data!")

if __name__ == "__main__":
    main()
