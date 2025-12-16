#!/usr/bin/env python3
"""
Test script to directly verify RAG system functionality
"""

import os
import sys
from pathlib import Path

# Add the project root to the Python path
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))

from rag_retriever import search_knowledge_base

print("🔍 Testing RAG system functionality...\n")

# Test 1: Simple query
print("Test 1: Query about 'humanoid robots'")
result1 = search_knowledge_base("humanoid robots")
print(f"Result length: {len(result1)} characters")
if "No relevant information found" not in result1:
    print("✅ Found relevant information!")
    print(f"Preview: {result1[:200]}...")
else:
    print("❌ No relevant information found")
print()

# Test 2: Query about 'actuators' 
print("Test 2: Query about 'actuators'")
result2 = search_knowledge_base("actuators")
print(f"Result length: {len(result2)} characters")
if "No relevant information found" not in result2:
    print("✅ Found relevant information!")
    print(f"Preview: {result2[:200]}...")
else:
    print("❌ No relevant information found")
print()

# Test 3: Query about 'control systems'
print("Test 3: Query about 'control systems'")
result3 = search_knowledge_base("control systems")
print(f"Result length: {len(result3)} characters")
if "No relevant information found" not in result3:
    print("✅ Found relevant information!")
    print(f"Preview: {result3[:200]}...")
else:
    print("❌ No relevant information found")
print()

# Test 4: Specific technical query
print("Test 4: Query about 'balance control'")
result4 = search_knowledge_base("balance control")
print(f"Result length: {len(result4)} characters")
if "No relevant information found" not in result4:
    print("✅ Found relevant information!")
    print(f"Preview: {result4[:200]}...")
else:
    print("❌ No relevant information found")
print()

print("📊 Summary:")
results = [result1, result2, result3, result4]
successful_queries = sum(1 for r in results if "No relevant information found" not in r)
print(f"✅ Successful queries: {successful_queries}/4")
print(f"❌ No results found: {4 - successful_queries}/4")

if successful_queries > 0:
    print("\n🎉 RAG system is working! It successfully retrieved content from the database.")
else:
    print("\n🤔 RAG system may not be finding relevant content.")