---
title: Fubuni Chat API
emoji: 🤖
colorFrom: blue
colorTo: purple
sdk: docker
app_port: 7860
pinned: false
---

# Fubuni Chat API

FastAPI backend for Fubuni AI chat assistant.

## Endpoints

- `GET /health` - Health check
- `POST /api/chat/sessions` - Create chat session
- `POST /api/chat/sessions/{session_id}/messages` - Send message
- `GET /api/chat/sessions/{session_id}/messages` - Get messages

## API Documentation

Visit `/docs` for Swagger UI documentation.
