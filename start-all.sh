#!/bin/bash

# Start all services for Fubuni Physical Humanoid Robotics Project
# This script starts the frontend, auth service, and backend API

set -e

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "🚀 Starting Fubuni Physical Humanoid Robotics Project..."
echo "=================================================="
echo ""

# Color codes for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Function to check if a port is in use
check_port() {
    local port=$1
    if lsof -Pi :$port -sTCP:LISTEN -t >/dev/null 2>&1; then
        return 0
    else
        return 1
    fi
}

# Function to kill process on a port
kill_port() {
    local port=$1
    local pid=$(lsof -ti:$port 2>/dev/null)
    if [ -n "$pid" ]; then
        echo -e "${YELLOW}Killing existing process on port $port (PID: $pid)${NC}"
        kill -9 $pid 2>/dev/null || true
        sleep 1
    fi
}

# Check and clean up existing processes
echo "🔍 Checking for existing processes..."
kill_port 3000  # Frontend
kill_port 4000  # Auth Service
kill_port 8000  # Backend API
echo ""

# Start Auth Service (port 4000)
echo "🔐 Starting Auth Service on port 4000..."
cd "$PROJECT_DIR/auth-service"
npm run dev > "$PROJECT_DIR/logs/auth-service.log" 2>&1 &
AUTH_PID=$!
echo -e "${GREEN}✓ Auth Service started (PID: $AUTH_PID)${NC}"
echo "  Log: $PROJECT_DIR/logs/auth-service.log"
echo ""

# Start Backend API (port 8000)
echo "🖥️  Starting Backend API on port 8000..."
cd "$PROJECT_DIR/backend"
source myenv/bin/activate
python -m uvicorn app.main:app --reload --host 0.0.0.0 --port 8000 > "$PROJECT_DIR/logs/backend.log" 2>&1 &
BACKEND_PID=$!
echo -e "${GREEN}✓ Backend API started (PID: $BACKEND_PID)${NC}"
echo "  Log: $PROJECT_DIR/logs/backend.log"
echo ""

# Start Frontend (port 3000)
echo "🌐 Starting Frontend on port 3000..."
cd "$PROJECT_DIR"
npm start > "$PROJECT_DIR/logs/frontend.log" 2>&1 &
FRONTEND_PID=$!
echo -e "${GREEN}✓ Frontend started (PID: $FRONTEND_PID)${NC}"
echo "  Log: $PROJECT_DIR/logs/frontend.log"
echo ""

# Wait for services to start
echo "⏳ Waiting for services to initialize..."
sleep 8

# Check service health
echo ""
echo "🏥 Health Check:"
echo "=================================================="

# Check Auth Service
if check_port 4000; then
    if curl -s https://ary-s-physical-humanoid-robotics-au.vercel.app/health | grep -q "healthy" 2>/dev/null; then
        echo -e "${GREEN}✓ Auth Service: Running${NC}"
        echo "  URL: http://localhost:4000"
    else
        echo -e "${YELLOW}⚠ Auth Service: Port open but not responding${NC}"
    fi
else
    echo -e "${RED}✗ Auth Service: Not running${NC}"
fi

# Check Backend API
if check_port 8000; then
    if curl -s http://localhost:8000/health | grep -q "healthy" 2>/dev/null; then
        echo -e "${GREEN}✓ Backend API: Running${NC}"
        echo "  URL: http://localhost:8000"
    else
        echo -e "${YELLOW}⚠ Backend API: Port open but not responding${NC}"
    fi
else
    echo -e "${RED}✗ Backend API: Not running${NC}"
fi

# Check Frontend
if check_port 3000; then
    echo -e "${GREEN}✓ Frontend: Running${NC}"
    echo "  URL: http://localhost:3000"
else
    echo -e "${RED}✗ Frontend: Not running${NC}"
fi

echo ""
echo "=================================================="
echo -e "${GREEN}🎉 All services started!${NC}"
echo ""
echo "📱 Application URLs:"
echo "  Frontend:     http://localhost:3000"
echo "  Chat Page:    http://localhost:3000/chat"
echo "  Auth Service: http://localhost:4000"
echo "  Backend API:  http://localhost:8000"
echo ""
echo "📊 Process IDs:"
echo "  Auth Service: $AUTH_PID"
echo "  Backend API:  $BACKEND_PID"
echo "  Frontend:     $FRONTEND_PID"
echo ""
echo "📝 Logs are available in: $PROJECT_DIR/logs/"
echo ""
echo "To stop all services, run: ./stop-all.sh"
echo "Or press Ctrl+C and manually kill the processes"
