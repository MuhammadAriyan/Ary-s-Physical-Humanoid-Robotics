"""
RAG (Retrieval-Augmented Generation) retriever module.
Uses Sentence Transformers (HuggingFace) for FREE embeddings - no OpenAI needed!
"""

import os
import logging
from typing import List
import hashlib

from qdrant_client import QdrantClient
from langchain_qdrant import QdrantVectorStore
from langchain_core.documents import Document
from langchain_huggingface import HuggingFaceEmbeddings

# Import settings
def get_settings():
    try:
        from app.config.settings import settings
        return settings
    except ImportError:
        return None

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Configuration from environment
RAG_ENABLED = os.getenv("RAG_ENABLED", "true").lower() == "true"

# Qdrant configuration
def get_qdrant_config():
    settings = get_settings()
    if settings:
        return settings.qdrant_url, settings.qdrant_api_key, settings.qdrant_collection_name
    # Fallback to env vars
    return (
        os.getenv("QDRANT_URL", "http://localhost:6333"),
        os.getenv("QDRANT_API_KEY", ""),
        os.getenv("QDRANT_COLLECTION_NAME", "robotics_docs")
    )

# Initialize global instances
_vector_store = None
_embeddings = None

# Query cache
_query_cache = {}
_MAX_CACHE_SIZE = 100

def get_embeddings():
    """Get or create HuggingFace embeddings (FREE, no API key needed)"""
    global _embeddings
    if _embeddings is None:
        logger.info("Initializing HuggingFace BGE embeddings (BAAI/bge-large-en-v1.5)...")
        _embeddings = HuggingFaceEmbeddings(
            model_name="BAAI/bge-large-en-v1.5",
            model_kwargs={'device': 'cpu'},
            encode_kwargs={'normalize_embeddings': True}
        )
        logger.info("HuggingFace BGE embeddings initialized successfully (1024 dims)")
    return _embeddings

def get_vector_store():
    """Get or create the Qdrant vector store"""
    global _vector_store

    if not RAG_ENABLED:
        logger.info("RAG is disabled via RAG_ENABLED environment variable")
        return None

    if _vector_store is not None:
        return _vector_store

    logger.info("Initializing Qdrant vector store...")

    qdrant_url, qdrant_api_key, qdrant_collection_name = get_qdrant_config()

    if not qdrant_url:
        logger.error("QDRANT_URL not configured")
        return None

    try:
        embeddings = get_embeddings()

        # Connect to Qdrant using QdrantVectorStore
        _vector_store = QdrantVectorStore.from_existing_collection(
            embedding=embeddings,
            collection_name=qdrant_collection_name,
            url=qdrant_url,
            api_key=qdrant_api_key if qdrant_api_key else None,
        )

        logger.info("Qdrant vector store initialized successfully!")
        return _vector_store

    except Exception as e:
        logger.error(f"Error initializing Qdrant vector store: {e}")
        return None

def initialize_retriever():
    """Initialize the RAG retriever - for backwards compatibility"""
    store = get_vector_store()
    if store:
        return store.as_retriever(search_kwargs={"k": 5})
    return None

def get_relevant_documents(query: str) -> List[Document]:
    """Retrieve relevant documents for a query"""
    # Check cache
    query_hash = hashlib.md5(query.encode('utf-8')).hexdigest()
    if query_hash in _query_cache:
        logger.info(f"Cache hit for query: {query[:50]}...")
        return _query_cache[query_hash]

    store = get_vector_store()
    if store is None:
        logger.warning("Vector store not available")
        return []

    try:
        # Use similarity_search directly on the vector store
        documents = store.similarity_search(query, k=5)
        logger.info(f"Retrieved {len(documents)} documents for: {query[:50]}...")

        # Cache results
        if len(_query_cache) < _MAX_CACHE_SIZE:
            _query_cache[query_hash] = documents

        return documents
    except Exception as e:
        logger.error(f"Error retrieving documents: {e}")
        return []

def get_relevant_documents_with_scores(query: str, top_k: int = 5) -> List[tuple]:
    """Retrieve documents with similarity scores"""
    store = get_vector_store()
    if store is None:
        return []

    try:
        results = store.similarity_search_with_score(query, k=top_k)
        return results
    except Exception as e:
        logger.error(f"Error retrieving documents with scores: {e}")
        return []

def search_knowledge_base(query: str) -> str:
    """
    Search the knowledge base - main function used by the agent.
    Uses FREE Sentence Transformers embeddings!
    """
    documents = get_relevant_documents(query)

    if not documents:
        return "No relevant information found in knowledge base."

    # Format results
    sources = []
    for doc in documents:
        source = doc.metadata.get('source', 'Unknown')
        title = doc.metadata.get('title', '')

        source_info = f"Source: {source}"
        if title:
            source_info += f" - {title}"

        content = doc.page_content[:500] + "..." if len(doc.page_content) > 500 else doc.page_content
        sources.append(f"{source_info}\nContent: {content}")

    return "Relevant information found:\n\n" + "\n\n---\n\n".join(sources)
