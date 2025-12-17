from sqlmodel import SQLModel, Field
from typing import Optional
from datetime import datetime
import uuid


class ChatSessionBase(SQLModel):
    user_id: str
    title: Optional[str] = None


class ChatSession(ChatSessionBase, table=True):
    __tablename__ = "chat_sessions"

    id: str = Field(default_factory=lambda: str(uuid.uuid4()), primary_key=True)
    created_at: datetime = Field(default_factory=datetime.utcnow)
    last_interaction: datetime = Field(default_factory=datetime.utcnow)
    is_active: bool = True


class ChatMessageBase(SQLModel):
    sender: str  # 'user' or 'fubuni'
    content: str
    chat_session_id: str
    sequence_number: int


class ChatMessage(ChatMessageBase, table=True):
    __tablename__ = "chat_messages"

    id: str = Field(default_factory=lambda: str(uuid.uuid4()), primary_key=True)
    timestamp: datetime = Field(default_factory=datetime.utcnow)


# Import the new translation model
from .translation import ChapterTranslationDB