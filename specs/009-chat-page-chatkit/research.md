# Research: Chat Page with Documentation Integration

**Feature**: 009-chat-page-chatkit
**Date**: 2025-12-20

## Research Tasks

### 1. OpenAI ChatKit Integration with React 19

**Question**: How to integrate @openai/chat-kit-react with Docusaurus/React 19?

**Decision**: Use @openai/chat-kit-react components with custom message handling

**Rationale**:
- ChatKit provides pre-built components (MessageList, MessageInput, Thread)
- Works with React 18+ (React 19 compatible)
- Allows custom backend integration via onSend handlers
- No vendor lock-in - just UI components

**Alternatives Considered**:
- Build custom chat UI: Rejected - more work, existing FubuniChat could be reused but ChatKit provides better UX
- Use existing FubuniChat: Partial - can reuse styling patterns but ChatKit gives better message threading

**Implementation Notes**:
```tsx
import { Thread, MessageList, MessageInput } from '@openai/chat-kit-react';

// Custom message handler connects to our backend
const handleSend = async (message: string) => {
  const response = await fetch('/api/chat', { ... });
  return response.json();
};
```

### 2. OpenAI Agents SDK Structured Output

**Question**: How to use output_type parameter for structured responses?

**Decision**: Use Pydantic BaseModel with Agent's output_type parameter

**Rationale**:
- OpenAI Agents SDK v0.0.4+ supports output_type on Agent class
- Pydantic v2 models work directly
- Agent returns typed object instead of string
- Enables chapter/section references in responses

**Alternatives Considered**:
- JSON parsing from string: Rejected - error-prone, no type safety
- Custom post-processing: Rejected - SDK handles this natively

**Implementation Notes**:
```python
from pydantic import BaseModel
from typing import Optional

class AgentResponse(BaseModel):
    response: str
    chapter: Optional[str] = None
    section: Optional[str] = None
    should_navigate: bool = False

agent = Agent(
    name="Fubuni",
    instructions="...",
    tools=[...],
    output_type=AgentResponse
)
```

### 3. iframe Doc Embedding Strategy

**Question**: How to embed Docusaurus docs in iframe without security issues?

**Decision**: Use same-origin iframe with dynamic src based on chapter

**Rationale**:
- Same-origin policy allows iframe embedding
- Docusaurus builds static pages, each chapter has own URL
- Can update iframe src to navigate chapters
- No postMessage complexity needed

**Alternatives Considered**:
- Fetch and render MDX: Rejected - complex, loses doc styling
- React Portal: Rejected - doesn't solve navigation
- Link navigation: Rejected - loses chat context

**Implementation Notes**:
```tsx
const DOC_CHAPTERS = {
  'introduction-to-humanoid-robotics': '/docs/humanoid-robotics-course/introduction-to-humanoid-robotics',
  'sensors-and-perception': '/docs/humanoid-robotics-course/sensors-and-perception',
  // ...
};

<iframe
  src={`${baseUrl}${DOC_CHAPTERS[currentChapter]}`}
  style={{ width: '100%', height: '100%', border: 'none' }}
/>
```

### 4. Chapter Mapping for Agent Instructions

**Question**: How should agent identify which chapter to reference?

**Decision**: Keyword-based mapping in agent instructions

**Rationale**:
- Simple and deterministic
- No additional ML/classification needed
- Easy to maintain and update
- Works with existing RAG results

**Chapter Mappings**:
| Chapter ID | Keywords |
|------------|----------|
| introduction-to-humanoid-robotics | humanoid, robot basics, overview, what is, introduction |
| sensors-and-perception | sensors, camera, lidar, perception, vision, detect |
| actuators-and-movement | actuator, motor, servo, movement, joint, DOF |
| control-systems | control, PID, feedback, stability, loop |
| path-planning-and-navigation | navigation, path, planning, SLAM, trajectory |

### 5. Mobile Responsive Strategy

**Question**: How to handle split-screen on mobile?

**Decision**: CSS media query with toggle button

**Rationale**:
- Native CSS solution, no JS complexity
- Toggle button provides clear UX
- Chat-first default makes sense for conversational flow
- Smooth transitions via CSS

**Implementation Notes**:
```css
@media (max-width: 768px) {
  .container {
    grid-template-columns: 1fr;
  }
  .docViewer {
    display: none;
  }
  .docViewer.visible {
    display: block;
    position: fixed;
    inset: 0;
    z-index: 100;
  }
}
```

## Unresolved Items

None - all research questions resolved.

## Dependencies Identified

| Dependency | Version | Purpose |
|------------|---------|---------|
| @openai/chat-kit-react | latest | Chat UI components |
| Pydantic | v2.10+ | Structured output models |
| OpenAI Agents SDK | v0.0.4+ | output_type support |

## Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| ChatKit incompatible with React 19 | Low | High | Fallback to existing FubuniChat |
| iframe blocked by browser | Low | Medium | Same-origin ensures compatibility |
| Agent doesn't use output_type correctly | Medium | Medium | Test with verbose logging |
