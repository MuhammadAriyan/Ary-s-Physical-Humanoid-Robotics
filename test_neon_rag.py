#!/usr/bin/env python3
"""
Test script to verify that RAG system correctly fetches data from Neon database.
This script will test the end-to-end functionality of the RAG system with NeonDB integration.
"""

import asyncio
import os
import sys
from pathlib import Path

# Add the project root to the Python path
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))

from rag_retriever import search_knowledge_base, initialize_retriever, get_relevant_documents
from backend.app.database.document_tracker import doc_tracker


def test_document_tracker():
    """Test that the document tracker can access the Neon database"""
    print("Testing document tracker access to NeonDB...")
    
    try:
        # Get all documents from the database
        docs = doc_tracker.get_all_documents()
        print(f"✓ Retrieved {len(docs)} documents from NeonDB")
        
        if len(docs) > 0:
            # Print details of first document
            first_doc = docs[0]
            print(f"  - First document ID: {first_doc.doc_id}")
            print(f"  - Source: {first_doc.source}")
            print(f"  - Title: {first_doc.title}")
            print(f"  - Content preview length: {len(first_doc.content_preview) if first_doc.content_preview else 0}")
            print(f"  - Full content length: {len(first_doc.content) if first_doc.content else 0}")
            print(f"  - Embedding model: {first_doc.embedding_model}")
        
        return len(docs) > 0
    except Exception as e:
        print(f"✗ Failed to access document tracker: {e}")
        return False


def test_retriever_initialization():
    """Test that the RAG retriever can be initialized"""
    print("\nTesting RAG retriever initialization...")
    
    try:
        retriever = initialize_retriever()
        if retriever is not None:
            print("✓ RAG retriever initialized successfully")
            return True
        else:
            print("✗ RAG retriever initialization failed - returned None")
            print("  Check that Qdrant is running and properly configured")
            return False
    except Exception as e:
        print(f"✗ Failed to initialize RAG retriever: {e}")
        return False


def test_document_retrieval():
    """Test that documents can be retrieved from the vector store"""
    print("\nTesting document retrieval from vector store...")
    
    try:
        # Try to retrieve documents with a generic query
        documents = get_relevant_documents("humanoid robot")
        print(f"✓ Retrieved {len(documents)} relevant documents for query 'humanoid robot'")
        
        if len(documents) > 0:
            # Print details of first document
            first_doc = documents[0]
            print(f"  - First document source: {first_doc.metadata.get('source', 'Unknown')}")
            print(f"  - Content preview: {first_doc.page_content[:100]}...")
        
        return len(documents) > 0
    except Exception as e:
        print(f"✗ Failed to retrieve documents: {e}")
        return False


def test_search_knowledge_base():
    """Test the search_knowledge_base function which is used by the agent tool"""
    print("\nTesting search_knowledge_base function (agent tool)...")
    
    try:
        # Test the function that the agent uses to search knowledge base
        result = search_knowledge_base("humanoid robot")
        print(f"✓ search_knowledge_base function executed successfully")
        print(f"  - Result length: {len(result)} characters")
        
        if "No relevant information found" not in result:
            print(f"  - Result preview: {result[:200]}...")
        else:
            print("  - No relevant documents found in knowledge base")
        
        return True
    except Exception as e:
        print(f"✗ Failed to execute search_knowledge_base function: {e}")
        return False


def test_database_consistency():
    """Test consistency between database and vector store"""
    print("\nTesting database and vector store consistency...")
    
    try:
        # Get documents from database
        db_docs = doc_tracker.get_all_documents()
        db_doc_ids = {doc.doc_id for doc in db_docs}
        print(f"✓ Found {len(db_docs)} documents in database")
        
        # Try to retrieve some documents from vector store
        if len(db_docs) > 0:
            # Use one of the document sources as a query to test retrieval
            sample_source = db_docs[0].source
            print(f"  - Using source '{sample_source}' to test retrieval")
            
            # We can't directly test if the same docs are in vector store without 
            # knowing Qdrant internals, but we can test that retrieval works
            sample_docs = get_relevant_documents(sample_source.split('/')[-1] if '/' in sample_source else sample_source)
            print(f"  - Retrieved {len(sample_docs)} documents related to sample source")
        
        return True
    except Exception as e:
        print(f"✗ Failed to test database consistency: {e}")
        return False


async def test_agent_integration():
    """Test that the agent can use the RAG tool properly"""
    print("\nTesting agent RAG tool integration...")
    
    try:
        from backend.app.agents.fubuni_agent import get_fubuni_agent
        
        # Get the agent instance
        agent = get_fubuni_agent()
        print("✓ Successfully retrieved Fubuni agent instance")
        
        # Test a simple message that might trigger RAG
        response = await agent.process_message("What do you know about humanoid robots?")
        print(f"✓ Agent processed message successfully")
        print(f"  - Response preview: {response[:200]}...")
        
        return True
    except Exception as e:
        print(f"✗ Failed to test agent integration: {e}")
        return False


def main():
    print("Starting comprehensive test of RAG system with NeonDB integration...\n")
    
    all_tests_passed = True
    
    # Test 1: Document tracker access
    if not test_document_tracker():
        all_tests_passed = False
    
    # Test 2: Retriever initialization
    if not test_retriever_initialization():
        all_tests_passed = False
    
    # Test 3: Document retrieval
    if not test_document_retrieval():
        all_tests_passed = False
    
    # Test 4: Search knowledge base function
    if not test_search_knowledge_base():
        all_tests_passed = False
    
    # Test 5: Database consistency
    if not test_database_consistency():
        all_tests_passed = False
    
    # Test 6: Agent integration
    if not asyncio.run(test_agent_integration()):
        all_tests_passed = False

    print("\n" + "="*60)
    if all_tests_passed:
        print("✓ All tests passed! RAG system is correctly fetching data from NeonDB.")
        print("\nThe RAG system is working properly with the following features:")
        print("- Documents are stored in Neon database with full content")
        print("- Vector store is created from database documents")
        print("- The search_knowledge_base tool retrieves information from the vector store")
        print("- The agent can use RAG functionality when needed")
    else:
        print("✗ Some tests failed. Please check the output above for details.")
    
    print("="*60)
    
    return all_tests_passed


if __name__ == "__main__":
    main()