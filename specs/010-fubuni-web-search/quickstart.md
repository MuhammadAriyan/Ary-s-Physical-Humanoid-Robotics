# Quickstart: Fubuni Web Search Integration

**Feature**: 010-fubuni-web-search
**Date**: 2025-12-21

## Prerequisites

- Python 3.12+ installed
- Node.js 20+ installed
- Backend virtual environment activated
- Frontend dependencies installed

## Quick Setup

### 1. Install Backend Dependency

```bash
cd backend
source myenv/bin/activate  # or your venv
pip install duckduckgo-search>=6.0.0
```

### 2. Verify Installation

```bash
python -c "from duckduckgo_search import DDGS; print('DuckDuckGo search installed!')"
```

### 3. Test Web Search Locally

```python
from duckduckgo_search import DDGS

with DDGS() as ddgs:
    results = list(ddgs.text("humanoid robot 2024", max_results=3))
    for r in results:
        print(f"Title: {r['title']}")
        print(f"URL: {r['href']}")
        print(f"Snippet: {r['body'][:100]}...")
        print("---")
```

## Implementation Steps

### Step 1: Add `search_web` Tool (Backend)

Edit `backend/app/agents/fubuni_agent.py`:

```python
from duckduckgo_search import DDGS

@function_tool
async def search_web(query: str) -> str:
    """
    Search the web using DuckDuckGo. ONLY use this tool if:
    1. search_knowledge_base returned no relevant results
    2. The user's message does NOT contain "use rag"

    Returns web search results with source URLs.
    """
    try:
        with DDGS() as ddgs:
            results = list(ddgs.text(query, max_results=5))

        if not results:
            return "No web search results found for this query."

        formatted = []
        for r in results:
            formatted.append(f"**{r['title']}**\n{r['body']}\nSource: {r['href']}")

        return "\n\n---\n\n".join(formatted)
    except Exception as e:
        return "Web search is temporarily unavailable."
```

### Step 2: Update Agent Tools List

In `FubuniAgent.__init__`:

```python
self.agent = Agent(
    name="Fubuni",
    instructions="""...(updated instructions)...""",
    tools=[
        search_knowledge_base,        # Priority 1
        search_knowledge_base_detailed, # Priority 2
        search_web,                    # Priority 3 (NEW)
        get_robotics_info,            # Priority 4 (fallback)
    ],
    output_type=AgentResponse,
)
```

### Step 3: Update Agent Instructions

Add to agent instructions:

```
## WEB SEARCH RULES:
- ONLY use search_web if search_knowledge_base returns no useful results
- NEVER use search_web if user's message contains "use rag"
- When using web search, ALWAYS cite sources with URLs
- Prefix web-sourced responses with: "🌐 **Source: Web Search**"
```

### Step 4: Add Frontend Panel (React)

In `src/pages/chat.tsx`, add WebSourcesPanel component:

```tsx
function WebSourcesPanel({
  sources,
  onClose
}: {
  sources: WebSearchResult[];
  onClose: () => void;
}) {
  return (
    <div className={styles.webSourcesPanel}>
      <button className={styles.closeSourcesButton} onClick={onClose}>
        <CloseIcon />
      </button>
      <h3>Web Sources</h3>
      <ul>
        {sources.map((source, i) => (
          <li key={i}>
            <a href={source.url} target="_blank" rel="noopener noreferrer">
              {source.title}
            </a>
            <p>{source.snippet}</p>
          </li>
        ))}
      </ul>
    </div>
  );
}
```

### Step 5: Add CSS Styles

In `src/pages/chat.module.css`:

```css
/* Web Sources Panel - slides from right */
.webSourcesPanel {
  position: absolute;
  right: 0;
  top: 0;
  width: 30%;
  height: 100%;
  transform: translateX(100%);
  opacity: 0;
  transition: transform 0.4s ease-out, opacity 0.3s ease-out;
}

.withSources .webSourcesPanel {
  transform: translateX(0);
  opacity: 1;
}

/* Triple layout */
.withAll {
  grid-template-columns: 40% 30% 30%;
}
```

## Testing

### Test 1: RAG Priority

Ask a question covered in documentation:
```
User: "What are humanoid robot sensors?"
Expected: Uses search_knowledge_base, returns doc content
```

### Test 2: Web Search Fallback

Ask about something NOT in documentation:
```
User: "What is Tesla Optimus Gen 3?"
Expected: RAG fails → uses search_web → returns web results with URLs
```

### Test 3: RAG Override

Force RAG-only mode:
```
User: "use rag Tell me about Boston Dynamics Spot"
Expected: Only uses RAG, never triggers web search
```

### Test 4: UI Panel Behavior

1. Send message that triggers web search
2. Verify sources panel slides in from right
3. Click X button to close
4. Send another message with "use rag"
5. Verify no sources panel appears

## Troubleshooting

### "DuckDuckGo rate limit"
- Wait a few seconds between requests
- Reduce `max_results` to 3

### "Web search not triggering"
- Check agent instructions include search_web tool
- Verify RAG is returning "no results" for the query

### "Sources panel not showing"
- Check `used_web_search` is true in API response
- Verify `web_sources` array is not empty
- Check CSS classes are applied correctly
