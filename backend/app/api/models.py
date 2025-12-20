from pydantic import BaseModel, Field
from typing import Optional, List
from datetime import datetime


class ChatRequest(BaseModel):
    message: str
    session_id: Optional[str] = None
    stream: bool = True


class WebSearchResult(BaseModel):
    """Single web search result from DuckDuckGo"""
    title: str = Field(..., max_length=200)
    url: str
    snippet: str = Field(..., max_length=500)


class ChatResponse(BaseModel):
    """T008: Updated with chapter navigation and web search fields"""
    response: str
    session_id: str
    timestamp: datetime
    chapter: Optional[str] = None  # Chapter to navigate to
    section: Optional[str] = None  # Section anchor within chapter
    should_navigate: bool = False  # Whether UI should auto-navigate
    error: Optional[str] = None
    # Web search fields
    web_sources: Optional[List[WebSearchResult]] = None
    used_web_search: bool = False


class StreamEvent(BaseModel):
    type: str  # 'text', 'tool_call', 'complete'
    content: str
    timestamp: datetime


class ChatMessageResponse(BaseModel):
    id: str
    sender: str
    content: str
    timestamp: datetime
    chat_session_id: str
    sequence_number: int


class ChatSessionResponse(BaseModel):
    id: str
    user_id: str
    created_at: datetime
    last_interaction: datetime
    is_active: bool
    title: Optional[str] = None