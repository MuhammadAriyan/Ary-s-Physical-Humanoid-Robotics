from fastapi import APIRouter
from pydantic import BaseModel
import json

router = APIRouter()


class ChatRequest(BaseModel):
    message: str
    session_id: str = None


class ChatResponse(BaseModel):
    response: str
    session_id: str = None


@router.post("/chat/test", response_model=ChatResponse)
async def test_chat_endpoint(chat_request: ChatRequest):
    """
    Simple test endpoint that returns a mock response
    """
    mock_response = f"Hello! I'm Fubuni, your AI assistant for Physical Humanoid Robotics. I see you're asking about: '{chat_request.message}'. This is a test response to verify the backend is working properly. The chat system is functional and ready for integration with real AI models."

    return ChatResponse(response=mock_response, session_id="test-session-123")
