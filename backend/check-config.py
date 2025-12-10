#!/usr/bin/env python3

import os
from dotenv import load_dotenv

# Load environment
load_dotenv()

OPENROUTER_API_KEY = os.getenv("OPENROUTER_API_KEY")
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")

print("=== API Configuration Status ===")
print(
    f"OpenRouter API Key: {'✅ Set' if OPENROUTER_API_KEY and not OPENROUTER_API_KEY.startswith('sk-or-v1-xxxxxxxx') else '❌ Not set or placeholder'}"
)
print(f"Gemini API Key: {'✅ Set' if GEMINI_API_KEY else '❌ Not set'}")

if OPENROUTER_API_KEY and not OPENROUTER_API_KEY.startswith("sk-or-v1-xxxxxxxx"):
    print("\n🚀 OpenRouter configuration is ready!")
    print("Model: amazon/nova-2-lite-v1:free")
    print("Base URL: https://openrouter.ai/api/v1")
elif GEMINI_API_KEY:
    print("\n⚠️  Using Gemini as fallback (may have quota issues)")
else:
    print("\n❌ No valid API keys configured - will use mock responses")
