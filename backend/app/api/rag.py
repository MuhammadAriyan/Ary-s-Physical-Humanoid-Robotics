from fastapi import APIRouter, HTTPException
import logging
import sys
import os
from pydantic import BaseModel, Field
from typing import Optional
import time

# Add parent directory to path for rag_retriever import
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

try:
    from rag_retriever import get_relevant_documents_with_scores, RAG_ENABLED
except ImportError:
    RAG_ENABLED = False
    def get_relevant_documents_with_scores(query, top_k):
        return []

logger = logging.getLogger(__name__)
router = APIRouter()

class RAGSearchRequest(BaseModel):
    query: str = Field(..., description="The search query to find relevant documents", max_length=1000)
    top_k: int = Field(5, ge=1, le=10, description="Number of top results to return")
    min_similarity: float = Field(0.5, ge=0.0, le=1.0, description="Minimum similarity score for results")
    include_metadata: bool = Field(True, description="Whether to include document metadata in results")

class DocumentResult(BaseModel):
    id: str
    content: str
    similarity_score: float
    metadata: dict

class RAGSearchResponse(BaseModel):
    query: str
    results: list[DocumentResult]
    total_results: int
    search_time_ms: float

@router.post("/rag/search", response_model=RAGSearchResponse)
async def rag_search_endpoint(request: RAGSearchRequest):
    """
    Search endpoint that queries the RAG system for relevant documents
    """
    start_time = time.time()

    # Check if RAG is enabled
    if not RAG_ENABLED:
        logger.info("RAG search attempted but RAG is disabled")
        return RAGSearchResponse(
            query=request.query,
            results=[],
            total_results=0,
            search_time_ms=0
        )

    try:
        # Get documents with similarity scores
        documents_with_scores = get_relevant_documents_with_scores(request.query, request.top_k)

        # Filter by minimum similarity score if specified
        if request.min_similarity > 0:
            documents_with_scores = [
                (doc, score) for doc, score in documents_with_scores
                if score >= request.min_similarity
            ]

        # Format results
        results = []
        for doc, score in documents_with_scores:
            doc_result = DocumentResult(
                id=doc.metadata.get('source', 'unknown'),  # Using source as ID for now
                content=doc.page_content,
                similarity_score=score,
                metadata=doc.metadata
            )
            results.append(doc_result)

        search_time_ms = (time.time() - start_time) * 1000

        return RAGSearchResponse(
            query=request.query,
            results=results,
            total_results=len(results),
            search_time_ms=search_time_ms
        )
    except Exception as e:
        logger.error(f"Error in RAG search: {e}")
        raise HTTPException(status_code=500, detail=f"Error performing RAG search: {str(e)}")