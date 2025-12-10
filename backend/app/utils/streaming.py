import json
from typing import AsyncGenerator, Dict, Any
from ..api.models import StreamEvent
from datetime import datetime


async def generate_sse_event(event_type: str, content: str) -> str:
    """
    Generate a Server-Sent Event string
    """
    event = StreamEvent(
        type=event_type,
        content=content,
        timestamp=datetime.utcnow()
    )
    data = f"data: {event.model_dump_json()}\n\n"
    return data


async def create_sse_stream(response: str) -> AsyncGenerator[str, None]:
    """
    Create an SSE stream from a response string, sending it token by token
    """
    # For simplicity, we'll send the response as chunks
    # In a real implementation, this would stream tokens as they're generated
    chunk_size = 10  # characters per chunk

    for i in range(0, len(response), chunk_size):
        chunk = response[i:i + chunk_size]
        yield await generate_sse_event("text", chunk)

    # Send completion event
    yield await generate_sse_event("complete", "Stream completed")