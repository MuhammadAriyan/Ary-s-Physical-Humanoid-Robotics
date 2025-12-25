"""
RAG (Retrieval-Augmented Generation) retriever module.
Uses Sentence Transformers (HuggingFace) for FREE embeddings - no OpenAI needed!
"""

import os
import logging
from typing import List, Dict, Any, Optional
from dataclasses import dataclass
import hashlib

from qdrant_client import QdrantClient
from langchain_qdrant import QdrantVectorStore
from langchain_core.documents import Document
from langchain_huggingface import HuggingFaceEmbeddings


# Chapter mapping - maps doc folder names to display names
CHAPTER_MAP = {
    "part-1-introduction": "introduction-to-humanoid-robotics",
    "part-2-ros2": "sensors-and-perception",  # Will be overridden by section detection
    "part-3-simulation": "control-systems",  # Will be overridden by section detection
    "part-4-hardware": "actuators-and-movement",  # Will be overridden by section detection
    "part-5-projects": "path-planning-and-navigation",  # Will be overridden by section detection
}

# T005: Valid chapters for documentation navigation
VALID_CHAPTERS = [
    "introduction-to-humanoid-robotics",
    "sensors-and-perception",
    "actuators-and-movement",
    "control-systems",
    "path-planning-and-navigation",
]


@dataclass
class DocSection:
    """Represents a documentation section with exact content"""
    title: str
    content: str
    url: str
    source_path: str


@dataclass
class DocReference:
    """Complete reference to a documentation section for navigation"""
    section: str
    chapter: str
    exact_lines: str
    url: str
    relevance_score: float


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

def detect_chapter_from_content(content: str) -> Optional[str]:
    """Detect the appropriate chapter based on content keywords."""
    content_lower = content.lower()

    # Check for chapter-related keywords
    if any(kw in content_lower for kw in ['introduction', 'basics', 'overview', 'what is', 'humanoid robot', 'getting started', 'course outline', 'part 1']):
        return "introduction-to-humanoid-robotics"
    elif any(kw in content_lower for kw in ['sensor', 'camera', 'lidar', 'perception', 'vision', 'detection', 'sensing', 'ros2', 'topic', 'publisher', 'subscriber', 'node']):
        return "sensors-and-perception"
    elif any(kw in content_lower for kw in ['actuator', 'motor', 'servo', 'movement', 'joint', 'dof', 'locomotion', 'unitree', 'go1', 'go2', 'aliengo']):
        return "actuators-and-movement"
    elif any(kw in content_lower for kw in ['control', 'pid', 'feedback', 'stability', 'loop', 'controller', 'simulation', 'gazebo', 'rviz']):
        return "control-systems"
    elif any(kw in content_lower for kw in ['navigation', 'path planning', 'slam', 'trajectory', 'waypoint', 'nav2', 'cartographer', 'navigation2']):
        return "path-planning-and-navigation"
    return None


def extract_exact_lines(content: str, query: str, max_lines: int = 10) -> str:
    """Extract the most relevant lines from content based on query keywords."""
    lines = content.split('\n')
    query_words = set(query.lower().split())

    # Score each line by query relevance
    line_scores = []
    for i, line in enumerate(lines):
        line_lower = line.lower()
        score = 0
        for word in query_words:
            if word in line_lower:
                score += 1
        # Bonus for lines with headings
        if line.strip().startswith('#'):
            score += 2
        # Bonus for code blocks
        if line.strip().startswith('```') or '```' in line:
            score += 1
        line_scores.append((i, score, line))

    # Sort by score and get top lines
    line_scores.sort(key=lambda x: -x[1])
    top_lines = [line for _, _, line in line_scores[:max_lines] if line.strip()]

    # Preserve order of top lines by original position
    if top_lines:
        top_line_indices = {line: i for i, line in enumerate(lines) if line in top_lines}
        top_lines.sort(key=lambda x: top_line_indices.get(x, 999))

    return '\n'.join(top_lines)


def format_doc_reference(doc: Document, query: str, score: float = 0.0) -> str:
    """Format a document as a detailed reference with exact lines."""
    source = doc.metadata.get('source', 'Unknown')
    url = doc.metadata.get('url', '')
    title = doc.metadata.get('title', '')
    section = doc.metadata.get('section', '')
    chapter = doc.metadata.get('part', '')

    # Detect chapter from content
    detected_chapter = detect_chapter_from_content(doc.page_content)
    if detected_chapter:
        chapter = detected_chapter

    # Format chapter name nicely
    chapter_display = chapter.replace('-', ' ').title() if chapter else 'General'

    # Extract exact relevant lines
    exact_lines = extract_exact_lines(doc.page_content, query, max_lines=15)

    # Build formatted reference
    score_str = f"{score:.2%}" if score > 0 else "N/A"
    ref = f"""
╔══════════════════════════════════════════════════════════════╗
║ DOCUMENT REFERENCE                                            ║
╠══════════════════════════════════════════════════════════════╣
║ Chapter: {chapter_display:<52} ║
║ Section: {section[:52] if section else 'General':<52} ║
║ File: {source[:52] if source else 'Unknown':<52} ║
║ Relevance: {score_str:<52} ║
╠══════════════════════════════════════════════════════════════╣
║ EXACT CONTENT FROM DOCUMENT:                                  ║
╠══════════════════════════════════════════════════════════════╣
{exact_lines}
╠══════════════════════════════════════════════════════════════╣
║ READ MORE: {url[:52] if url else 'N/A':<52} ║
╚══════════════════════════════════════════════════════════════╝"""
    return ref


def search_knowledge_base(query: str) -> str:
    """
    Search the knowledge base - main function used by the agent.
    Returns detailed documentation references with exact lines and chapter info.
    Uses FREE Sentence Transformers embeddings!
    """
    results = get_relevant_documents_with_scores(query, top_k=5)

    if not results:
        return "No relevant information found in knowledge base."

    # Format results with detailed references
    references = []
    for doc, score in results:
        ref = format_doc_reference(doc, query, score)
        references.append(ref)

    # Build summary
    summary = f"""
╔══════════════════════════════════════════════════════════════╗
║  📚 KNOWLEDGE BASE SEARCH RESULTS                             ║
║  Query: "{query}"                                             ║
╠══════════════════════════════════════════════════════════════╣
Found {len(references)} relevant documentation sections:

{chr(10).join(references)}

╔══════════════════════════════════════════════════════════════╗
║  NAVIGATION HINTS                                             ║
╠══════════════════════════════════════════════════════════════╣
Based on your query, you may want to read:
"""

    # Add chapter-specific hints
    for doc, score in results[:2]:  # Top 2 results
        chapter = detect_chapter_from_content(doc.page_content)
        if chapter:
            chapter_display = chapter.replace('-', ' ').title()
            url = doc.metadata.get('url', '')
            summary += f"  • {chapter_display}: {url}\n"

    summary += """
The exact lines above are pulled directly from the documentation.
Use the "READ MORE" link to see the full context.
"""

    return summary


def search_knowledge_base_structured(query: str) -> tuple:
    """
    Search the knowledge base and return structured data.
    Returns: (response_text, chapter, should_navigate)
    """
    results = get_relevant_documents_with_scores(query, top_k=3)

    if not results:
        return "No relevant information found in knowledge base.", None, False

    # Detect primary chapter from top result
    primary_chapter = None
    for doc, score in results:
        chapter = detect_chapter_from_content(doc.page_content)
        if chapter:
            primary_chapter = chapter
            break

    # Build response with exact lines
    response_parts = ["**Relevant Documentation:**\n"]
    for i, (doc, score) in enumerate(results):
        source = doc.metadata.get('source', 'Unknown')
        url = doc.metadata.get('url', '')
        section = doc.metadata.get('section', '')
        exact_lines = extract_exact_lines(doc.page_content, query, max_lines=8)

        response_parts.append(f"""
**[{i+1}] {source}** (Relevance: {score:.1%})
{section}
{exact_lines}
🔗 [Read full section]({url})
""")

    response_text = '\n'.join(response_parts)
    return response_text, primary_chapter, primary_chapter is not None
