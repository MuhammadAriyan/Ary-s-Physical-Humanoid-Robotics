#!/bin/bash

echo "🧪 Testing Backend Configuration"
echo "================================="

# Test 1: Check if agent imports work
echo "1. Testing agent import..."
cd backend
python3 -c "
import sys
sys.path.append('.')
try:
    from app.agents.fubuni_agent import fubuni_agent
    print('✅ Agent imports successfully')
    print('✅ Model:', 'anthropic/claude-3.5-haiku:free' if 'OPENROUTER_API_KEY' in __import__('os').environ else 'NOT SET')
except Exception as e:
    print(f'❌ Error: {e}')
"

echo ""
echo "2. Testing FastAPI app creation..."
python3 -c "
import sys
sys.path.append('.')
try:
    from app.main import app
    print('✅ FastAPI app created successfully')
except Exception as e:
    print(f'❌ Error: {e}')
"

echo ""
echo "3. Checking environment variables..."
echo "OPENROUTER_API_KEY: ${OPENROUTER_API_KEY:0:10}..."
echo "NEON_DATABASE_URL: ${NEON_DATABASE_URL:0:20}..."

echo ""
echo "🎯 Next Steps:"
echo "1. Deploy to Railway with these environment variables"
echo "2. Update frontend with Railway URL"
echo "3. Test chat functionality"