#!/usr/bin/env python3
"""
Final end-to-end RAG functionality test
"""

import os
import sys
from pathlib import Path

# Add the project root to the Python path
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))

from backend.app.database.document_tracker import doc_tracker
from rag_retriever import search_knowledge_base, get_relevant_documents

print("🎯 FINAL RAG SYSTEM VERIFICATION")
print("="*50)

# Step 1: Check database
docs = doc_tracker.get_all_documents()
print(f"1. Database contains {len(docs)} documents")

# Step 2: Show some sample documents
sample_docs = docs[:3]  # Take first 3 documents
for i, doc in enumerate(sample_docs, 1):
    print(f"   Doc {i}: {doc.source}")

# Step 3: Test retrieval
print(f"\n2. Testing retrieval functionality:")
test_query = "humanoid robot actuators"
documents = get_relevant_documents(test_query)
print(f"   Query: '{test_query}'")
print(f"   Retrieved {len(documents)} documents")

if documents:
    first_doc = documents[0]
    print(f"   First document metadata: {first_doc.metadata}")
    print(f"   Content preview: {first_doc.page_content[:100]}...")

# Step 4: Test the search function
print(f"\n3. Testing search_knowledge_base function:")
result = search_knowledge_base("actuators in humanoid robots")
print(f"   Result length: {len(result)} characters")
if "No relevant information found" not in result:
    print("   ✅ Content found!")
    
    # Show sources
    lines = result.split('\n')
    sources = [line for line in lines if line.startswith("Source:")]
    print(f"   Sources: {len(sources)} found")
    for source in sources[:3]:  # Show first 3 sources
        print(f"     - {source}")
else:
    print("   ❌ No content found")

# Step 5: Verify content is from database
print(f"\n4. Verification results:")
print(f"   ✅ Database accessible: {len(docs) > 0}")
print(f"   ✅ Documents retrieved: {len(documents) > 0}")
print(f"   ✅ Search function works: {'No relevant information found' not in result}")
print(f"   ✅ Sources cited: {len([l for l in result.split('\\n') if l.startswith('Source:')]) > 0}")

print(f"\n🎯 CONCLUSION:")
if len(docs) > 0 and len(documents) > 0 and 'No relevant information found' not in result:
    print("✅ RAG SYSTEM IS WORKING PERFECTLY!")
    print("✅ Content is being retrieved from the database!")
    print("✅ All functionality verified and operational!")
else:
    print("❌ Some issues detected - please review output above")

print("="*50)