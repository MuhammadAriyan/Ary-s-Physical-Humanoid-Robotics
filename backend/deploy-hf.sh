#!/bin/bash
# Hugging Face Spaces Deployment Script
# Usage: ./deploy-hf.sh YOUR_HF_USERNAME SPACE_NAME

set -e

HF_USER=${1:-"your-username"}
SPACE_NAME=${2:-"fubuni-chat-api"}
TEMP_DIR="/tmp/hf-deploy-$$"

echo "🚀 Deploying to Hugging Face Spaces: $HF_USER/$SPACE_NAME"

# Create temp directory
mkdir -p "$TEMP_DIR"

# Copy necessary files
echo "📦 Preparing files..."
cp Dockerfile.hf "$TEMP_DIR/Dockerfile"
cp README.hf.md "$TEMP_DIR/README.md"
cp requirements.txt "$TEMP_DIR/"
cp -r app "$TEMP_DIR/"

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
echo "   - NEON_DATABASE_URL"
echo "   - OPENROUTER_API_KEY"
echo "   - APP_ENV=production"

# Cleanup
rm -rf "$TEMP_DIR"
