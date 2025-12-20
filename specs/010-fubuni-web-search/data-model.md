# Data Model: Fubuni Web Search Integration

**Feature**: 010-fubuni-web-search
**Date**: 2025-12-21

## Entities

### WebSearchResult

Represents a single web search result from DuckDuckGo.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| title | string | Yes | Page title from search result |
| url | string | Yes | Full URL to the source page |
| snippet | string | Yes | Text excerpt/body from search result |

**Validation Rules**:
- `title`: Max 200 characters
- `url`: Must be valid URL format (https:// or http://)
- `snippet`: Max 500 characters

**Source Mapping** (from DuckDuckGo API):
- `title` ← `result.title`
- `url` ← `result.href`
- `snippet` ← `result.body`

### AgentResponse (Extended)

Existing model extended with web search fields.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| response | string | Yes | Agent's text response |
| chapter | string | No | Documentation chapter for navigation |
| section | string | No | Specific section within chapter |
| should_navigate | bool | No | Whether to auto-navigate to chapter |
| **web_sources** | WebSearchResult[] | No | **NEW**: Web search results used |
| **used_web_search** | bool | No | **NEW**: Flag if web search was used |

### ChatResponse (Extended)

API response model extended with web search fields.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| response | string | Yes | Agent's text response |
| session_id | string | Yes | Chat session identifier |
| timestamp | datetime | Yes | Response timestamp |
| chapter | string | No | Documentation chapter for navigation |
| section | string | No | Specific section within chapter |
| should_navigate | bool | No | Whether to auto-navigate to chapter |
| **web_sources** | WebSearchResult[] | No | **NEW**: Web search results array |
| **used_web_search** | bool | No | **NEW**: Flag if web search was used |

## State Transitions

### Web Sources Panel State

```
        ┌─────────────────────────────────────────────┐
        │                                             │
        ▼                                             │
    ┌───────┐    web_sources received    ┌─────────┐ │
    │Hidden │ ─────────────────────────► │ Visible │ │
    └───────┘                            └─────────┘ │
        ▲                                     │      │
        │         user clicks X               │      │
        └─────────────────────────────────────┘      │
                                                     │
        ┌───────────────────────────────────────────┘
        │  new message without web_sources
        │
        │  (panel remains in current state)
```

### Layout State Machine

```
                    ┌──────────────┐
                    │   chatOnly   │
                    └──────────────┘
                      ↙           ↘
        docs nav clicked           web_sources received
                    ↓                       ↓
            ┌───────────┐           ┌────────────┐
            │ withDocs  │           │ withSources│
            └───────────┘           └────────────┘
                    ↘           ↙
           both panels triggered
                    ↓
            ┌───────────┐
            │  withAll  │
            └───────────┘
```

## Relationships

```
ChatSession (existing)
    │
    ├── 1:N ─► ChatMessage (existing)
    │
    └── 1:N ─► ChatResponse
                    │
                    └── 0:N ─► WebSearchResult (embedded)
```

## Backend Pydantic Models

### New Model: WebSearchResult

```python
# backend/app/api/models.py

class WebSearchResult(BaseModel):
    """Single web search result from DuckDuckGo"""
    title: str = Field(..., max_length=200)
    url: str = Field(..., pattern=r'^https?://')
    snippet: str = Field(..., max_length=500)
```

### Extended Model: AgentResponse

```python
# backend/app/agents/fubuni_agent.py

class AgentResponse(BaseModel):
    """Structured agent response with navigation and web search hints"""
    response: str
    chapter: Optional[str] = None
    section: Optional[str] = None
    should_navigate: bool = False
    # NEW fields
    web_sources: Optional[list[WebSearchResult]] = None
    used_web_search: bool = False

    @field_validator('chapter')
    @classmethod
    def validate_chapter(cls, v):
        if v is not None and v not in VALID_CHAPTERS:
            return None
        return v
```

### Extended Model: ChatResponse

```python
# backend/app/api/models.py

class ChatResponse(BaseModel):
    """Chat endpoint response model"""
    response: str
    session_id: str
    timestamp: datetime
    chapter: Optional[str] = None
    section: Optional[str] = None
    should_navigate: bool = False
    # NEW fields
    web_sources: Optional[list[WebSearchResult]] = None
    used_web_search: bool = False
```

## Frontend TypeScript Interfaces

### New Interface: WebSearchResult

```typescript
// src/pages/chat.tsx

interface WebSearchResult {
  title: string;
  url: string;
  snippet: string;
}
```

### Extended Interface: BackendResponse

```typescript
// src/pages/chat.tsx

interface BackendResponse {
  response: string;
  chapter?: string;
  section?: string;
  should_navigate?: boolean;
  // NEW fields
  web_sources?: WebSearchResult[];
  used_web_search?: boolean;
}
```

## Constraints

1. **Web search limit**: Maximum 5 results per query
2. **Snippet length**: Truncated to 500 characters if longer
3. **URL validation**: Must start with http:// or https://
4. **RAG override**: If message contains "use rag", `web_sources` will always be empty
5. **Error state**: On web search failure, `web_sources = []` and `used_web_search = false`
