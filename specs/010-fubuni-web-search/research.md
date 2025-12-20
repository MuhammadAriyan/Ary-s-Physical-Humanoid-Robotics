# Research: Fubuni Web Search Integration

**Feature**: 010-fubuni-web-search
**Date**: 2025-12-21
**Status**: Complete

## Research Tasks

### R1: DuckDuckGo Search Library

**Question**: Best Python library for DuckDuckGo search integration?

**Decision**: `duckduckgo-search` library (pypi: duckduckgo-search)

**Rationale**:
- Most actively maintained DuckDuckGo search library
- No API key required - uses DuckDuckGo's public search
- Async support available (`AsyncDDGS`)
- Returns structured results with title, href, body
- Rate limiting handled gracefully

**Alternatives Considered**:
- `duckduckgo` (deprecated, no longer maintained)
- `ddg` (minimal features, limited results)
- Custom HTTP requests (unnecessary complexity)

**Usage Pattern**:
```python
from duckduckgo_search import DDGS

def search_web(query: str, max_results: int = 5) -> list[dict]:
    with DDGS() as ddgs:
        results = list(ddgs.text(query, max_results=max_results))
    return results

# Returns: [{"title": "...", "href": "...", "body": "..."}]
```

### R2: OpenAI Agents SDK Function Tool Pattern

**Question**: How to add new function tool to existing agent?

**Decision**: Use existing `@function_tool` decorator pattern

**Rationale**:
- Already used for `search_knowledge_base`, `search_knowledge_base_detailed`, `get_robotics_info`
- Consistent pattern across codebase
- Agent instructions control tool priority

**Implementation Reference** (from `fubuni_agent.py:110-126`):
```python
@function_tool
async def search_knowledge_base(query: str) -> str:
    """
    ALWAYS use this tool FIRST for ANY question...
    """
    result = rag_search_knowledge_base(query)
    return result
```

### R3: Agent Instructions for Tool Priority

**Question**: How to ensure web search is lowest priority?

**Decision**: Update agent instructions to explicitly order tool usage

**Rationale**:
- Current instructions already use priority language ("ALWAYS use this tool FIRST")
- Add explicit "ONLY use search_web AFTER search_knowledge_base returns no results"
- Add "NEVER use search_web if message contains 'use rag'"

**Current Priority** (from agent instructions):
1. `search_knowledge_base` - PRIMARY
2. `search_knowledge_base_detailed` - MORE DETAILED
3. `get_robotics_info` - LAST RESORT

**New Priority**:
1. `search_knowledge_base` - PRIMARY (RAG)
2. `search_knowledge_base_detailed` - MORE DETAILED (RAG)
3. `search_web` - SECONDARY (only if RAG has no results)
4. `get_robotics_info` - LAST RESORT (general knowledge)

### R4: CSS Grid Triple Layout

**Question**: Best CSS approach for 3-panel responsive layout?

**Decision**: CSS Grid with dynamic `grid-template-columns`

**Rationale**:
- Already using CSS Grid for current 2-panel layout
- Easy to add third column
- Smooth transitions with `transition: grid-template-columns`

**Implementation**:
```css
/* All three panels visible */
.withAll {
  grid-template-columns: 40% 30% 30%;
}

/* Web sources only (no docs) */
.withSources {
  grid-template-columns: 70% 30%;
}
```

### R5: Slide Animation from Right

**Question**: How to implement right-to-left slide animation?

**Decision**: Mirror existing docs panel pattern with `transform: translateX(100%)`

**Rationale**:
- Consistent with existing left-slide docs panel
- Uses same cubic-bezier easing for smooth animation
- Same close button pattern

**Reference** (from `chat.module.css:36-45`):
```css
/* Docs panel hidden state */
.chatOnly .docViewer {
  transform: translateX(-100%);  /* Slides left */
  opacity: 0;
}

/* Web sources panel hidden state (mirror) */
.chatOnly .webSourcesPanel {
  transform: translateX(100%);   /* Slides right */
  opacity: 0;
}
```

### R6: Response Schema Extension

**Question**: How to pass web search results from backend to frontend?

**Decision**: Add `web_sources` array field to existing `ChatResponse` Pydantic model

**Rationale**:
- Minimal API change
- Frontend can check `used_web_search` boolean
- `web_sources` array contains title, url, snippet for each result

**Backend Changes** (`app/api/models.py`):
```python
class WebSearchResult(BaseModel):
    title: str
    url: str
    snippet: str

class ChatResponse(BaseModel):
    response: str
    session_id: str
    timestamp: datetime
    chapter: Optional[str] = None
    section: Optional[str] = None
    should_navigate: bool = False
    # NEW
    web_sources: Optional[list[WebSearchResult]] = None
    used_web_search: bool = False
```

### R7: Error Handling for Web Search

**Question**: How to handle web search failures gracefully?

**Decision**: Catch exceptions in tool, return empty results, log warning

**Rationale**:
- Constitution requires no exposed errors to users
- Agent instructions handle "no results" case
- Fallback to general knowledge tool if web search fails

**Implementation**:
```python
@function_tool
async def search_web(query: str) -> str:
    """Search the web using DuckDuckGo..."""
    try:
        with DDGS() as ddgs:
            results = list(ddgs.text(query, max_results=5))
        if not results:
            return "No web search results found."
        # Format results...
    except Exception as e:
        logging.warning(f"Web search failed: {e}")
        return "Web search unavailable. Please try documentation search."
```

## Resolved Questions

| Question | Resolution |
|----------|------------|
| Which DuckDuckGo library? | `duckduckgo-search` (async support, maintained) |
| How to add function tool? | `@function_tool` decorator pattern |
| Tool priority enforcement? | Agent instructions with explicit ordering |
| Triple layout CSS? | CSS Grid with `grid-template-columns: 40% 30% 30%` |
| Slide animation direction? | `translateX(100%)` for right-side panel |
| API response format? | Add `web_sources: WebSearchResult[]` to ChatResponse |
| Error handling? | Try/catch with fallback message, log warning |

## Dependencies to Add

```txt
# backend/requirements.txt
duckduckgo-search>=6.0.0
```

## Files to Modify

| File | Change |
|------|--------|
| `backend/requirements.txt` | Add `duckduckgo-search>=6.0.0` |
| `backend/app/api/models.py` | Add `WebSearchResult`, update `ChatResponse` |
| `backend/app/agents/fubuni_agent.py` | Add `search_web` tool, update `AgentResponse` |
| `backend/app/api/chat.py` | Pass web_sources to response |
| `src/pages/chat.tsx` | Add WebSourcesPanel, handle triple layout |
| `src/pages/chat.module.css` | Add `.webSourcesPanel`, `.withAll`, `.withSources` styles |
