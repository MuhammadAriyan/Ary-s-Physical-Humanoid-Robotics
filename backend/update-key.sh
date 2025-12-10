#!/bin/bash

# Script to update OpenRouter API key
echo "Current OpenRouter key in .env:"
grep OPENROUTER_API_KEY .env

echo ""
echo "To update your OpenRouter API key, edit the .env file:"
echo "OPENROUTER_API_KEY=your_real_openrouter_key_here"
echo ""
echo "Then restart the backend with:"
echo "pkill -f uvicorn"
echo "cd backend && source venv/bin/activate && uvicorn app.main:app --reload --host 0.0.0.0 --port 8000 &"