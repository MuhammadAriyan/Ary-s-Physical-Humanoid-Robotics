# 🚀 Deploy Backend to Railway

## Quick Steps:

1. **Go to [railway.app](https://railway.app)**
2. **Sign up with GitHub**
3. **Click "New Project" → "Deploy from GitHub repo"**
4. **Select**: `Ary-s-Physical-Humanoid-Robotics`
5. **Set Root Directory**: `backend/`
6. **Add Environment Variables**:
   ```bash
   NEON_DATABASE_URL=postgresql://your-neon-db-url
   OPENROUTER_API_KEY=your-openrouter-api-key
   APP_ENV=production
   BACKEND_CORS_ORIGINS=["https://muhammadariyan.github.io"]
   ```
7. **Click "Deploy"**

## After Deployment:

1. **Get your Railway URL** from dashboard
2. **Update frontend** in `src/theme/FubuniChatInjector/index.js`:
   ```javascript
   const backendUrl = 'https://your-app-name.railway.app';
   ```
3. **Test endpoints**:
   ```bash
   curl https://your-app.railway.app/health
   ```

## ✅ Backend Features:
- Real Fubuni AI agent with OpenAI Agents SDK
- PostgreSQL database for chat history
- Streaming chat responses
- Session management
- Health checks

Your backend is production-ready! 🎉