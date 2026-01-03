from fastapi import APIRouter, Depends, HTTPException
from fastapi.responses import StreamingResponse
from sqlmodel import Session, select
from typing import Optional
import uuid
from datetime import datetime
from ..models import ChatSession, ChatMessage
from ..api.models import ChatRequest, ChatResponse
from ..agents.fubuni_agent import get_fubuni_agent
from ..config.database import get_session
from ..utils.streaming import create_sse_stream
from ..config.settings import settings
from ..middleware.auth import AuthUser, require_auth, get_current_user


router = APIRouter()


@router.get("/chat/sessions")
async def get_chat_sessions(
    session: Session = Depends(get_session),
    current_user: AuthUser = Depends(require_auth),
):
    """
    Get all chat sessions for the authenticated user.
    Requires authentication. Returns sessions ordered by last_interaction DESC.
    """
    try:
        # Filter sessions by authenticated user, order by most recent first
        chat_sessions = session.exec(
            select(ChatSession)
            .where(ChatSession.user_id == current_user.id)
            .order_by(ChatSession.last_interaction.desc())
        ).all()

        # Add message count for each session
        sessions_with_count = []
        for chat_session in chat_sessions:
            message_count = len(session.exec(
                select(ChatMessage).where(ChatMessage.chat_session_id == chat_session.id)
            ).all())
            sessions_with_count.append({
                "id": chat_session.id,
                "title": chat_session.title,
                "created_at": chat_session.created_at,
                "last_interaction": chat_session.last_interaction,
                "is_active": chat_session.is_active,
                "message_count": message_count,
            })

        return {"sessions": sessions_with_count, "total": len(sessions_with_count)}
    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Error retrieving chat sessions: {str(e)}"
        )


@router.post("/chat/sessions")
async def create_chat_session(
    session: Session = Depends(get_session),
    current_user: AuthUser = Depends(require_auth),
):
    """
    Create a new empty chat session for the authenticated user.
    Requires authentication.
    """
    try:
        chat_session = ChatSession(
            user_id=current_user.id,
            title=None,  # Will be auto-generated from first message
        )
        session.add(chat_session)
        session.commit()
        session.refresh(chat_session)

        return {
            "id": chat_session.id,
            "title": chat_session.title,
            "created_at": chat_session.created_at,
            "last_interaction": chat_session.last_interaction,
            "is_active": chat_session.is_active,
            "message_count": 0,
        }
    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Error creating chat session: {str(e)}"
        )


@router.delete("/chat/sessions/{session_id}")
async def delete_chat_session(
    session_id: str,
    session: Session = Depends(get_session),
    current_user: AuthUser = Depends(require_auth),
):
    """
    Delete a chat session and all its messages.
    Requires authentication and verifies ownership.
    """
    try:
        # Check if session exists and belongs to the authenticated user
        chat_session = session.exec(
            select(ChatSession).where(
                ChatSession.id == session_id,
                ChatSession.user_id == current_user.id,
            )
        ).first()

        if not chat_session:
            raise HTTPException(
                status_code=404,
                detail="Chat session not found or you don't have permission to delete it"
            )

        # Delete all messages in the session first
        messages = session.exec(
            select(ChatMessage).where(ChatMessage.chat_session_id == session_id)
        ).all()
        for message in messages:
            session.delete(message)

        # Delete the session
        session.delete(chat_session)
        session.commit()

        return {"success": True, "deleted_session_id": session_id}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Error deleting chat session: {str(e)}"
        )


@router.get("/chat/history/{session_id}")
async def get_chat_history(
    session_id: str,
    session: Session = Depends(get_session),
    current_user: AuthUser = Depends(require_auth),
):
    """
    Get chat history for a specific session.
    Requires authentication and validates ownership.
    """
    try:
        # Check if session exists and belongs to the authenticated user
        chat_session = session.exec(
            select(ChatSession).where(
                ChatSession.id == session_id,
                ChatSession.user_id == current_user.id,
            )
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
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Error retrieving chat history: {str(e)}"
        )


@router.post("/chat", response_model=ChatResponse)
async def chat_endpoint(
    chat_request: ChatRequest,
    session: Session = Depends(get_session),
    current_user: AuthUser = Depends(get_current_user),
):
    """
    Chat endpoint that processes user messages and returns Fubuni's response.
    Supports both authenticated and anonymous users.
    Authenticated users have their conversations saved to the database.
    Anonymous users can use the RAG functionality without saving.
    """
    # Use authenticated user ID if available, otherwise generate a temporary ID for anonymous users
    user_id = current_user.id if current_user else f"anon_{uuid.uuid4()}"

    # Get or create a chat session
    chat_session: Optional[ChatSession] = None
    if chat_request.session_id and current_user:
        # Only try to find existing session if user is authenticated (prevent unauthorized access to others' sessions)
        chat_session = session.exec(
            select(ChatSession).where(
                ChatSession.id == chat_request.session_id,
                ChatSession.user_id == current_user.id,  # Use actual user ID for auth check
            )
        ).first()

    # If no session provided or found, create a new one
    if not chat_session:
        # For anonymous users, create temporary session (won't be saved to DB permanently)
        chat_session = ChatSession(
            user_id=user_id,
            title=chat_request.message[:50]
            if len(chat_request.message) > 50
            else chat_request.message,
        )
        # Only add to session if user is authenticated (to save to DB)
        if current_user:
            session.add(chat_session)
            session.commit()
            session.refresh(chat_session)
        else:
            # For anonymous users, generate ID but don't save to DB
            chat_session.id = str(uuid.uuid4())

    # Save user message only if user is authenticated
    if current_user:
        user_message = ChatMessage(
            sender="user",
            content=chat_request.message,
            chat_session_id=chat_session.id,
            sequence_number=1,  # Will need to implement proper sequencing in a full implementation
        )
        session.add(user_message)
        session.commit()

    # Process the message with the Fubuni agent (T009: Structured output)
    try:
        fubuni_agent = get_fubuni_agent()
        agent_response = await fubuni_agent.process_message(
            chat_request.message, chat_session.id
        )

        # Extract response text from AgentResponse
        response_text = agent_response.response if hasattr(agent_response, 'response') else str(agent_response)

        # Save Fubuni's response only if user is authenticated
        if current_user:
            fubuni_message = ChatMessage(
                sender="fubuni",
                content=response_text,
                chat_session_id=chat_session.id,
                sequence_number=2,  # Will need to implement proper sequencing in a full implementation
            )
            session.add(fubuni_message)
            session.commit()

            # Update session last interaction
            chat_session.last_interaction = datetime.utcnow()
            session.add(chat_session)
            session.commit()

        # Return structured response with chapter navigation fields
        return ChatResponse(
            response=response_text,
            session_id=chat_session.id,
            timestamp=datetime.utcnow(),
            chapter=agent_response.chapter if hasattr(agent_response, 'chapter') else None,
            section=agent_response.section if hasattr(agent_response, 'section') else None,
            should_navigate=agent_response.should_navigate if hasattr(agent_response, 'should_navigate') else False,
        )
    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Error processing message: {str(e)}"
        )


@router.post("/chat/stream")
async def chat_stream_endpoint(
    chat_request: ChatRequest,
    session: Session = Depends(get_session),
    current_user: AuthUser = Depends(get_current_user),
):
    """
    Streaming chat endpoint that returns Fubuni's response as Server-Sent Events.
    Supports both authenticated and anonymous users.
    Authenticated users have their conversations saved to the database.
    Anonymous users can use the RAG functionality without saving.
    """
    # Use authenticated user ID if available, otherwise generate a temporary ID for anonymous users
    user_id = current_user.id if current_user else f"anon_{uuid.uuid4()}"

    # Get or create a chat session (similar to above)
    chat_session: Optional[ChatSession] = None
    if chat_request.session_id and current_user:
        # Only allow access to sessions owned by this user if authenticated
        chat_session = session.exec(
            select(ChatSession).where(
                ChatSession.id == chat_request.session_id,
                ChatSession.user_id == current_user.id,
            )
        ).first()

    if not chat_session:
        chat_session = ChatSession(
            user_id=user_id,
            title=chat_request.message[:50]
            if len(chat_request.message) > 50
            else chat_request.message,
        )
        # Only add to session if user is authenticated (to save to DB)
        if current_user:
            session.add(chat_session)
            session.commit()
            session.refresh(chat_session)
        else:
            # For anonymous users, generate ID but don't save to DB
            chat_session.id = str(uuid.uuid4())

    # Save user message only if user is authenticated
    if current_user:
        user_message = ChatMessage(
            sender="user",
            content=chat_request.message,
            chat_session_id=chat_session.id,
            sequence_number=1,
        )
        session.add(user_message)
        session.commit()

    # Capture session ID before session closes
    session_id = chat_session.id

    # Process the message with the Fubuni agent
    try:
        # Create streaming response
        async def generate():
            full_response = ""
            fubuni_agent = get_fubuni_agent()
            async for chunk in fubuni_agent.process_message_streamed(
                chat_request.message, session_id
            ):
                full_response += chunk
                async for sse_chunk in create_sse_stream(chunk):
                    yield sse_chunk

            # Save the complete response to database only if user is authenticated
            if current_user:
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
