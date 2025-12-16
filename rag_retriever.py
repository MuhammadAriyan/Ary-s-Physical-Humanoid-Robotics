"""
RAG (Retrieval-Augmented Generation) retriever module.
This module provides functionality to retrieve relevant documents from the vector store
based on user queries, using contextual compression for better relevance.
"""

import os
import logging
from typing import List, Optional
import hashlib
from datetime import datetime

from langchain_openai import OpenAIEmbeddings, ChatOpenAI
from qdrant_client import QdrantClient
from qdrant_client.http.models import Distance, VectorParams
from langchain_qdrant import Qdrant
from langchain.retrievers import ContextualCompressionRetriever
from langchain.retrievers.document_compressors import LLMChainExtractor
from langchain.docstore.document import Document

# Import settings (but avoid circular import by importing inside functions)
def get_settings():
    from backend.app.config.settings import settings
    return settings

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

import hashlib
from datetime import datetime

# Get configuration from environment
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
RAG_ENABLED = os.getenv("RAG_ENABLED", "true").lower() == "true"

# Qdrant configuration - getting settings from function
def get_qdrant_config():
    settings = get_settings()
    return settings.qdrant_url, settings.qdrant_api_key, settings.qdrant_collection_name

# Neon database configuration (for metadata storage)
NEON_DATABASE_URL = os.getenv("NEON_DATABASE_URL")

def get_docs_hash(docs_path="./docs"):
    """Generate a hash of all document files to detect changes"""
    hash_md5 = hashlib.md5()

    if not os.path.exists(docs_path):
        return None

    for root, dirs, files in os.walk(docs_path):
        for file in sorted(files):  # Sort to ensure consistent ordering
            file_path = os.path.join(root, file)
            if os.path.isfile(file_path):
                # Update hash with file path and modification time
                hash_md5.update(file_path.encode('utf-8'))
                hash_md5.update(str(os.path.getmtime(file_path)).encode('utf-8'))

    return hash_md5.hexdigest()

def should_rebuild_vectorstore(docs_path="./docs"):
    """Check if vector store needs to be rebuilt based on document changes"""
    # Get Qdrant configuration
    qdrant_url, qdrant_api_key, qdrant_collection_name = get_qdrant_config()

    # Connect to Qdrant to check collection existence
    try:
        if qdrant_api_key:
            client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
        else:
            client = QdrantClient(url=qdrant_url)

        # List collections to check if our collection exists
        collections = client.get_collections()
        collection_names = [collection.name for collection in collections.collections]

        # If the collection doesn't exist, we need to build it
        if qdrant_collection_name not in collection_names:
            return True

        # Check if we have a stored hash of the docs (stored in a config file or DB)
        # In a real implementation, this would be stored in Neon or as a Qdrant collection metadata
        import tempfile
        import os

        # For now, using a temporary solution to store document hash
        hash_file_path = os.path.join(tempfile.gettempdir(), f".{qdrant_collection_name}_docs_hash")
        if not os.path.exists(hash_file_path):
            return True

        # Read the stored hash
        with open(hash_file_path, 'r') as f:
            stored_hash = f.read().strip()

        # Get the current docs hash
        current_hash = get_docs_hash(docs_path)

        # Return True if hashes don't match (meaning docs have changed)
        return stored_hash != current_hash
    except Exception as e:
        logger.error(f"Error checking if vector store needs rebuild: {e}")
        return True  # Rebuild if we can't connect to Qdrant

# Initialize global retriever instance
_retriever = None

# Query cache for performance optimization
_query_cache = {}
_MAX_CACHE_SIZE = 100  # Maximum number of cached queries

def initialize_retriever():
    """Initialize the RAG retriever with Qdrant and compression"""
    global _retriever

    # Check if RAG is enabled
    if not RAG_ENABLED:
        logger.info("RAG functionality is disabled via RAG_ENABLED environment variable")
        return None

    if _retriever is not None:
        return _retriever

    if not OPENAI_API_KEY:
        logger.error("OPENAI_API_KEY environment variable is required for RAG functionality")
        return None

    logger.info("Initializing RAG retriever with Qdrant...")

    # Get Qdrant configuration
    qdrant_url, qdrant_api_key, qdrant_collection_name = get_qdrant_config()

    # Initialize OpenAI embeddings
    embeddings = OpenAIEmbeddings(
        model="text-embedding-3-large",
        openai_api_key=OPENAI_API_KEY
    )

    # Connect to Qdrant
    try:
        if qdrant_api_key:
            client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
        else:
            # Try connection without API key (for local Qdrant)
            client = QdrantClient(url=qdrant_url)

        # Create or connect to the Qdrant collection
        # Check if collection exists first
        import qdrant_client.http.exceptions as qdrant_exceptions
        collection_exists = False
        try:
            client.get_collection(qdrant_collection_name)
            collection_exists = True
        except qdrant_exceptions.UnexpectedResponse:
            # Collection doesn't exist, will create it
            collection_exists = False

        if not collection_exists:
            logger.warning(f"Qdrant collection {qdrant_collection_name} does not exist. Please run create_vectorstore.py first.")
            return None

        # Create Qdrant vector store
        qdrant_store = Qdrant(
            client=client,
            collection_name=qdrant_collection_name,
            embeddings=embeddings,
        )

        # Create base retriever
        base_retriever = qdrant_store.as_retriever(
            search_kwargs={"k": 6}  # Retrieve top 6 most relevant documents
        )

        # Initialize the LLM for contextual compression
        llm = ChatOpenAI(
            model="gpt-4o-mini",
            temperature=0,
            openai_api_key=OPENAI_API_KEY
        )

        # Create the compressor using the LLM
        compressor = LLMChainExtractor.from_llm(llm)

        # Create the contextual compression retriever
        _retriever = ContextualCompressionRetriever(
            base_retriever=base_retriever,
            base_compressor=compressor
        )

        logger.info("RAG retriever with Qdrant initialized successfully")
        return _retriever
    except Exception as e:
        logger.error(f"Error initializing Qdrant-based RAG retriever: {e}")
        return None

def get_relevant_documents(query: str) -> List[Document]:
    """
    Retrieve relevant documents for a given query

    Args:
        query: The user's query string

    Returns:
        List of relevant Document objects
    """
    # Check cache first
    query_hash = hashlib.md5(query.encode('utf-8')).hexdigest()
    if query_hash in _query_cache:
        logger.info(f"Retrieved documents for query from cache: {query[:50]}...")
        return _query_cache[query_hash]

    retriever = initialize_retriever()

    if retriever is None:
        logger.warning("RAG retriever not available, returning empty results")
        return []

    try:
        documents = retriever.get_relevant_documents(query)
        logger.info(f"Retrieved {len(documents)} relevant documents for query: {query[:50]}...")

        # Add to cache if cache has space
        if len(_query_cache) < _MAX_CACHE_SIZE:
            _query_cache[query_hash] = documents

        return documents
    except Exception as e:
        logger.error(f"Error retrieving documents: {e}")
        return []

def get_relevant_documents_with_scores(query: str, top_k: int = 5) -> List[tuple[Document, float]]:
    """
    Retrieve relevant documents with similarity scores for a given query

    Args:
        query: The user's query string
        top_k: Number of top results to return

    Returns:
        List of tuples containing (Document, similarity_score)
    """
    # Initialize the base vector store retriever to access similarity_search_with_score
    if initialize_retriever() is None:
        logger.warning("RAG retriever not available, returning empty results")
        return []

    # Get the base retriever which should be a Qdrant instance
    try:
        # Qdrant has a similarity_search_with_score method
        if hasattr(_retriever.base_retriever.vectorstore, 'similarity_search_with_score'):
            results_with_scores = _retriever.base_retriever.vectorstore.similarity_search_with_score(
                query, k=top_k
            )
            return results_with_scores
        else:
            # Fallback: just return documents without real scores
            documents = _retriever.base_retriever.get_relevant_documents(query)
            # Return documents with dummy scores
            return [(doc, 0.0) for doc in documents[:top_k]]
    except Exception as e:
        logger.error(f"Error retrieving documents with scores: {e}")
        return []

def format_documents_as_context(documents: List[Document]) -> str:
    """
    Format retrieved documents into a context string for the LLM
    
    Args:
        documents: List of retrieved Document objects
        
    Returns:
        Formatted context string
    """
    if not documents:
        return ""
    
    formatted_docs = []
    for i, doc in enumerate(documents):
        source = doc.metadata.get("source", "Unknown")
        content = doc.page_content
        
        # Create a formatted entry for each document
        formatted_doc = f"""
Document {i+1}:
Source: {source}
Content: {content}

"""
        formatted_docs.append(formatted_doc)
    
    return "Relevant Information from Documentation:\n" + "".join(formatted_docs)

def get_rag_response(query: str) -> Optional[str]:
    """
    Get a RAG-enhanced response for a query
    
    Args:
        query: The user's query string
        
    Returns:
        Formatted response with context from documents, or None if no relevant docs found
    """
    documents = get_relevant_documents(query)
    
    if not documents:
        logger.info("No relevant documents found for query")
        return None
    
    context = format_documents_as_context(documents)
    return context

# For backward compatibility with the existing agent structure
def search_knowledge_base(query: str) -> str:
    """
    Search the knowledge base for relevant information.
    This function maintains compatibility with the existing agent tool interface.

    Args:
        query: The query string to search for

    Returns:
        Formatted string with relevant information and sources
    """
    documents = get_relevant_documents(query)

    if not documents or len(documents) == 0:
        return "No relevant information found in knowledge base."

    # Format the response with detailed sources
    sources = []
    for doc in documents:
        source = doc.metadata.get('source', 'Unknown')
        page_number = doc.metadata.get('page', None)  # Available for PDFs
        title = doc.metadata.get('title', None)

        # Construct detailed source information
        source_info = f"Source: {source}"
        if page_number is not None:
            source_info += f", Page: {page_number}"
        if title:
            source_info += f", Title: {title}"

        content_preview = doc.page_content[:300] + "..." if len(doc.page_content) > 300 else doc.page_content
        sources.append(f"{source_info}\nContent: {content_preview}")

    return f"Relevant information found:\n" + "\n\n".join(sources)