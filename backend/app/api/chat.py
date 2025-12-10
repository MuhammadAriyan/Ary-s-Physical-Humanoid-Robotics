from fastapi import APIRouter, Depends, HTTPException
from fastapi.responses import StreamingResponse
from sqlmodel import Session, select
from typing import Optional
import uuid
from datetime import datetime
from ..models import ChatSession, ChatMessage
from ..api.models import ChatRequest, ChatResponse
from ..agents.fubuni_agent import fubuni_agent
from ..config.database import get_session
from ..utils.streaming import create_sse_stream
from ..config.settings import settings


router = APIRouter()


@router.get("/chat/sessions")
async def get_chat_sessions(session: Session = Depends(get_session)):
    """
    Get all chat sessions for a user
    """
    try:
        # For now, return all sessions (in a real implementation, this would be filtered by user)
        chat_sessions = session.exec(select(ChatSession)).all()
        return chat_sessions
    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Error retrieving chat sessions: {str(e)}"
        )


@router.get("/chat/history/{session_id}")
async def get_chat_history(session_id: str, session: Session = Depends(get_session)):
    """
    Get chat history for a specific session
    """
    try:
        # Check if session exists
        chat_session = session.exec(
            select(ChatSession).where(ChatSession.id == session_id)
        ).first()

        if not chat_session:
            raise HTTPException(status_code=404, detail="Chat session not found")

        # Get all messages for this session ordered by sequence number
        messages = session.exec(
            select(ChatMessage)
            .where(ChatMessage.chat_session_id == session_id)
            .order_by(ChatMessage.sequence_number)
        ).all()

        return {"session": chat_session, "messages": messages}
    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Error retrieving chat history: {str(e)}"
        )


@router.post("/chat", response_model=ChatResponse)
async def chat_endpoint(
    chat_request: ChatRequest, session: Session = Depends(get_session)
):
    """
    Chat endpoint that processes user messages and returns Fubuni's response
    """
    # Get or create a chat session
    chat_session: Optional[ChatSession] = None
    if chat_request.session_id:
        # Try to find existing session
        chat_session = session.exec(
            select(ChatSession).where(ChatSession.id == chat_request.session_id)
        ).first()

    # If no session provided or found, create a new one
    if not chat_session:
        chat_session = ChatSession(
            user_id=f"anonymous_{uuid.uuid4()}",
            title=chat_request.message[:50]
            if len(chat_request.message) > 50
            else chat_request.message,
        )
        session.add(chat_session)
        session.commit()
        session.refresh(chat_session)

    # Save user message
    user_message = ChatMessage(
        sender="user",
        content=chat_request.message,
        chat_session_id=chat_session.id,
        sequence_number=1,  # Will need to implement proper sequencing in a full implementation
    )
    session.add(user_message)
    session.commit()

    # Process the message with the Fubuni agent
    try:
        agent_response = await fubuni_agent.process_message(
            chat_request.message, chat_session.id
        )

        # Save Fubuni's response
        fubuni_message = ChatMessage(
            sender="fubuni",
            content=agent_response,
            chat_session_id=chat_session.id,
            sequence_number=2,  # Will need to implement proper sequencing in a full implementation
        )
        session.add(fubuni_message)
        session.commit()

        # Update session last interaction
        chat_session.last_interaction = datetime.utcnow()
        session.add(chat_session)
        session.commit()

        return ChatResponse(
            response=agent_response,
            session_id=chat_session.id,
            timestamp=datetime.utcnow(),
        )
    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Error processing message: {str(e)}"
        )


@router.post("/chat/stream")
async def chat_stream_endpoint(
    chat_request: ChatRequest, session: Session = Depends(get_session)
):
    """
    Streaming chat endpoint that returns Fubuni's response as Server-Sent Events
    """
    # Get or create a chat session (similar to above)
    chat_session: Optional[ChatSession] = None
    if chat_request.session_id:
        chat_session = session.exec(
            select(ChatSession).where(ChatSession.id == chat_request.session_id)
        ).first()

    if not chat_session:
        chat_session = ChatSession(
            user_id=f"anonymous_{uuid.uuid4()}",
            title=chat_request.message[:50]
            if len(chat_request.message) > 50
            else chat_request.message,
        )
        session.add(chat_session)
        session.commit()
        session.refresh(chat_session)

    # Save user message
    user_message = ChatMessage(
        sender="user",
        content=chat_request.message,
        chat_session_id=chat_session.id,
        sequence_number=1,
    )
    session.add(user_message)
    session.commit()

    # Process the message with the Fubuni agent
    try:
        # Create streaming response
        async def generate():
            full_response = ""
            async for chunk in fubuni_agent.process_message_streamed(
                chat_request.message, chat_session.id
            ):
                full_response += chunk
                async for sse_chunk in create_sse_stream(chunk):
                    yield sse_chunk

            # Save the complete response to database
            fubuni_message = ChatMessage(
                sender="fubuni",
                content=full_response,
                chat_session_id=chat_session.id,
                sequence_number=2,
            )
            session.add(fubuni_message)
            session.commit()

            # Update session last interaction
            chat_session.last_interaction = datetime.utcnow()
            session.add(chat_session)
            session.commit()

        return StreamingResponse(generate(), media_type="text/event-stream")
    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Error processing message: {str(e)}"
        )
