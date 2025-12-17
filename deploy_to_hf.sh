#!/bin/bash
# Deployment script to Hugging Face Spaces for Fubuni Chat with Authentication
# Usage: ./deploy_to_hf.sh YOUR_HF_USERNAME SPACE_NAME

set -e

HF_USER=${1:-"your-username"}
SPACE_NAME=${2:-"fubuni-chat-auth"}
TEMP_DIR="/tmp/hf-deploy-$$"

echo "🚀 Deploying Fubuni Chat with Authentication to Hugging Face Spaces: $HF_USER/$SPACE_NAME"

# Create temp directory
mkdir -p "$TEMP_DIR"

# Copy backend files
cp backend/Dockerfile.hf.auth "$TEMP_DIR/Dockerfile"
cp backend/README.hf.md "$TEMP_DIR/README.md"
cp backend/requirements.txt "$TEMP_DIR/"
cp -r backend/app "$TEMP_DIR/"
cp backend/rag_retriever.py "$TEMP_DIR/"

# Copy auth service files
cp -r auth-service/package*.json "$TEMP_DIR/"
cp -r auth-service/src "$TEMP_DIR/src"
cp -r auth-service/tsconfig.json "$TEMP_DIR/"

# Create .gitignore
cat > "$TEMP_DIR/.gitignore" << 'EOF'
__pycache__/
*.pyc
.env
.env.*
*.db
myenv/
venv/
.venv/
node_modules/
EOF

cd "$TEMP_DIR"

echo "📤 Uploading to Hugging Face..."
# Check if huggingface-cli is installed
if ! command -v huggingface-cli &> /dev/null; then
    echo "Installing huggingface_hub..."
    pip install huggingface_hub -q
fi

# Upload using HF CLI
huggingface-cli upload "$HF_USER/$SPACE_NAME" . . --repo-type space

echo "✅ Deployment complete!"
echo "🔗 View your space at: https://huggingface.co/spaces/$HF_USER/$SPACE_NAME"
echo ""
echo "⚠️  Don't forget to set secrets in Space Settings:"
echo "   - NEON_DATABASE_URL (your Neon PostgreSQL connection string)"
echo "   - OPENROUTER_API_KEY (for chat functionality)"
echo "   - BETTER_AUTH_SECRET (32+ character secret for auth)"
echo ""
echo "📝 For local development after deployment:"
echo "   The auth service will run on port 4000 internally"
echo "   The backend API will run on port 7860 (Hugging Face standard)"

# Cleanup
rm -rf "$TEMP_DIR"