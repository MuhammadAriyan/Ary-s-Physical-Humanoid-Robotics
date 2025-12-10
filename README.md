# Ary's Physical Humanoid Robotics

An interactive educational platform for humanoid robotics featuring an AI-powered chatbot assistant.

## 🚀 Features

- **AI Chatbot**: Fubuni assistant powered by OpenAI Agents SDK
- **Modern UI**: Black & white glassmorphism design
- **Educational Content**: Comprehensive robotics curriculum
- **Full-Stack**: React frontend with FastAPI backend
- **Database**: PostgreSQL integration for chat history

## 🌐 Live Demo

- **Frontend**: https://muhammadariyan.github.io/Ary-s-Physical-Humanoid-Robotics
- **Backend**: Deploy on Railway or Vercel (see deployment guide)

## 🛠️ Quick Start

### Frontend
```bash
npm install
npm start
```

### Backend
```bash
cd backend
pip install -r requirements.txt
uvicorn app.main:app --reload
```

## 🚀 Deployment

### Frontend (GitHub Pages)
- Automatically deployed on push to main branch
- Uses GitHub Actions CI/CD

### Backend (Railway Recommended)
1. Go to [railway.app](https://railway.app)
2. Connect GitHub repository
3. Set root path to `backend/`
4. Add environment variables:
   ```bash
   NEON_DATABASE_URL=your-neon-db-url
   OPENROUTER_API_KEY=your-openrouter-api-key
   APP_ENV=production
   BACKEND_CORS_ORIGINS=["https://muhammadariyan.github.io"]
   ```

## 📁 Project Structure

```
├── src/                    # React frontend
│   └── components/FubuniChat/  # Chat components
├── backend/                 # FastAPI backend
│   ├── app/
│   │   ├── api/             # API endpoints
│   │   ├── agents/          # AI agent logic
│   │   └── models/          # Database models
└── docs/                   # Educational content
```

## 🤖 Chatbot Features

- **Robotics Expert**: Specialized in humanoid robotics
- **Streaming Responses**: Real-time chat
- **Session Management**: Conversation history
- **Mobile Responsive**: Works on all devices

## 📄 License

MIT License
