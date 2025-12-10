# Data Model: Fubuni Chat Bubble

## Core Entities

### ChatMessage
Represents a single message in the conversation stored in Neon Postgres database

**Fields**:
- `id: string` - Unique identifier for the message (Primary Key)
- `sender: 'user' | 'fubuni'` - Indicates whether the message is from user or Fubuni
- `content: string` - The text content of the message
- `timestamp: datetime` - When the message was created/sent
- `chat_session_id: string` - Foreign key reference to the parent chat session
- `sequence_number: integer` - Order of the message in the conversation

### ChatSession
Represents a user's chat session stored in Neon Postgres database

**Fields**:
- `id: string` - Unique identifier for the session (Primary Key)
- `user_id: string` - Identifier for the user (could be session-based or anonymous)
- `created_at: datetime` - When the session was started
- `last_interaction: datetime` - When the last message was sent
- `is_active: boolean` - Whether the session is currently active
- `title: string` - Optional title for the session (derived from first message)

### FubuniAgent
Represents the AI agent that processes user queries

**Fields**:
- `id: string` - Unique identifier for the agent
- `name: string` - Name of the agent ("Fubuni")
- `system_prompt: string` - The system instructions for the agent
- `model_config: object` - Configuration for the AI model
- `tools: List[Tool]` - List of available tools for the agent

### Tool
Represents a function/tool available to the agent

**Fields**:
- `name: string` - Name of the tool
- `description: string` - What the tool does
- `parameters: object` - Input parameters for the tool

## API Models

### ChatRequest
**Fields**:
- `message: string` - The user's message/query
- `session_id: string` - Optional session identifier for context
- `stream: boolean` - Whether to stream the response (default: true)

### ChatResponse
**Fields**:
- `response: string` - The agent's response
- `session_id: string` - Session identifier
- `timestamp: datetime` - When the response was generated
- `error: string` - Optional error message if the request failed

### StreamEvent
For SSE streaming responses
**Fields**:
- `type: 'text' | 'tool_call' | 'complete'` - Type of event
- `content: string` - The content of the event
- `timestamp: datetime` - When the event occurred