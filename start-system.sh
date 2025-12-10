#!/bin/bash

echo "🚀 Starting Ary's Physical Humanoid Robotics Chat System"
echo "=================================================="

# Start Backend
echo "1. Starting Backend Server..."
cd backend
source venv/bin/activate
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000 &
BACKEND_PID=$!
echo "✅ Backend started (PID: $BACKEND_PID) on http://localhost:8000"

# Wait for backend to initialize
sleep 3

# Test backend health
echo "2. Checking Backend Health..."
if curl -s http://localhost:8000/health | grep -q "healthy"; then
    echo "✅ Backend is healthy and ready"
else
    echo "❌ Backend health check failed"
fi

# Start Frontend
echo "3. Starting Frontend Server..."
cd /home/ary/Dev/Ary-s-Physical-Humanoid-Robotics
npm start &
FRONTEND_PID=$!
echo "✅ Frontend started (PID: $FRONTEND_PID) on http://localhost:3000"

# Wait for frontend to initialize
sleep 5

echo ""
echo "🎉 System is ready!"
echo "==================="
echo "📱 Frontend: http://localhost:3000"
echo "🤖 Backend:  http://localhost:8000"
echo "💬 Chat: Click the Fubuni bubble on the website"
echo ""
echo "To stop both servers:"
echo "kill $BACKEND_PID $FRONTEND_PID"
echo ""
echo "To check status:"
echo "curl http://localhost:8000/health"