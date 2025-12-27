# Project Startup Guide

## Quick Start

### Start All Services
```bash
./start-all.sh
```

This will start:
- **Frontend** (Docusaurus) on `http://localhost:3000`
- **Auth Service** on `http://localhost:4000`
- **Backend API** on `http://localhost:8000`

### Stop All Services
```bash
./stop-all.sh
```

## Services Overview

### 1. Frontend (Port 3000)
- **Technology**: Docusaurus 3 + React 19 + TypeScript
- **Main Pages**:
  - Home: `http://localhost:3000`
  - Chat: `http://localhost:3000/chat`
- **Log File**: `logs/frontend.log`

### 2. Auth Service (Port 4000)
- **Technology**: Node.js + Express + Better Auth
- **Purpose**: Handles user authentication (OAuth, email/password)
- **Health Check**: `http://localhost:4000/health`
- **Log File**: `logs/auth-service.log`

### 3. Backend API (Port 8000)
- **Technology**: Python 3.11 + FastAPI + OpenRouter
- **Purpose**: Chat API with RAG capabilities
- **Health Check**: `http://localhost:8000/health`
- **API Docs**: `http://localhost:8000/docs`
- **Log File**: `logs/backend.log`

## Manual Start (Individual Services)

### Frontend
```bash
npm start
```

### Auth Service
```bash
cd auth-service
npm run dev
```

### Backend API
```bash
cd backend
source myenv/bin/activate
python -m uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

## Troubleshooting

### Port Already in Use
The `start-all.sh` script automatically kills any processes on ports 3000, 4000, and 8000 before starting.

If you need to manually kill a process:
```bash
# Find process on port
lsof -ti:3000

# Kill process
kill -9 $(lsof -ti:3000)
```

### Check Service Status
```bash
# Check if services are running
curl http://localhost:3000  # Frontend
curl http://localhost:4000/health  # Auth Service
curl http://localhost:8000/health  # Backend API
```

### View Logs
```bash
# View logs in real-time
tail -f logs/frontend.log
tail -f logs/auth-service.log
tail -f logs/backend.log
```

## Environment Variables

### Backend (.env in backend/)
- `OPENROUTER_API_KEY`: OpenRouter API key
- `NEON_DATABASE_URL`: PostgreSQL connection string
- `QDRANT_URL`: Qdrant vector database URL
- `JWT_SECRET`: Shared secret with auth service

### Auth Service (.env in auth-service/)
- `DATABASE_URL`: PostgreSQL connection string (same as backend)
- `JWT_SECRET`: Shared secret with backend
- OAuth provider credentials (GitHub, Google, etc.)

## Development Workflow

1. **Start all services**: `./start-all.sh`
2. **Open chat page**: Navigate to `http://localhost:3000/chat`
3. **Sign in**: Use the auth modal to sign in
4. **Start chatting**: Send messages to Fubuni
5. **Check logs**: Monitor `logs/` directory for any issues
6. **Stop services**: Run `./stop-all.sh` when done

## Production Deployment

For production deployment:
- Frontend: Deploy to Vercel/Netlify
- Auth Service: Deploy to Koyeb/Railway
- Backend API: Deploy to Hugging Face Spaces or Railway

Update CORS settings in both services to allow your production domains.
