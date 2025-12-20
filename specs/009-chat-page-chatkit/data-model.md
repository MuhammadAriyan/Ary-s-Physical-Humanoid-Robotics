# Data Model: Chat Page with Documentation Integration

**Feature**: 009-chat-page-chatkit
**Date**: 2025-12-20

## Entities

### AgentResponse (Backend - NEW)

Structured response from Fubuni agent including documentation navigation hints.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| response | string | Yes | The text response from the agent |
| chapter | string | No | Chapter identifier to navigate to (e.g., "sensors-and-perception") |
| section | string | No | Section anchor within chapter (e.g., "types-of-sensors") |
| should_navigate | boolean | No | Whether UI should auto-navigate to chapter (default: false) |

**Validation Rules**:
- `response` must be non-empty
- `chapter` must be one of: introduction-to-humanoid-robotics, sensors-and-perception, actuators-and-movement, control-systems, path-planning-and-navigation
- `should_navigate` only true when `chapter` is provided

**Pydantic Model**:
```python
from pydantic import BaseModel, field_validator
from typing import Optional

VALID_CHAPTERS = [
    "introduction-to-humanoid-robotics",
    "sensors-and-perception",
    "actuators-and-movement",
    "control-systems",
    "path-planning-and-navigation",
]

class AgentResponse(BaseModel):
    response: str
    chapter: Optional[str] = None
    section: Optional[str] = None
    should_navigate: bool = False

    @field_validator('chapter')
    @classmethod
    def validate_chapter(cls, v):
        if v is not None and v not in VALID_CHAPTERS:
            return None  # Fallback to no navigation
        return v
```

---

### ChatResponse (Backend - MODIFIED)

Updated API response model with chapter navigation fields.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| response | string | Yes | Agent's text response |
| session_id | string | Yes | Chat session identifier |
| timestamp | datetime | Yes | Response timestamp |
| chapter | string | No | Chapter to navigate to |
| section | string | No | Section anchor |
| should_navigate | boolean | No | Auto-navigate flag |
| error | string | No | Error message if any |

---

### ChatPageState (Frontend - NEW)

React state for the /chat page component.

| Field | Type | Initial | Description |
|-------|------|---------|-------------|
| currentChapter | string | "introduction-to-humanoid-robotics" | Currently displayed chapter |
| messages | Message[] | [] | Chat message history |
| isLoading | boolean | false | Chat request in progress |
| isMobileDocVisible | boolean | false | Mobile: show doc panel |

---

### ChapterMapping (Frontend - NEW)

Static mapping of chapter IDs to documentation URLs.

| Chapter ID | URL Path |
|------------|----------|
| introduction-to-humanoid-robotics | /docs/humanoid-robotics-course/introduction-to-humanoid-robotics |
| sensors-and-perception | /docs/humanoid-robotics-course/sensors-and-perception |
| actuators-and-movement | /docs/humanoid-robotics-course/actuators-and-movement |
| control-systems | /docs/humanoid-robotics-course/control-systems |
| path-planning-and-navigation | /docs/humanoid-robotics-course/path-planning-and-navigation |

**TypeScript Definition**:
```typescript
const DOC_CHAPTERS: Record<string, string> = {
  'introduction-to-humanoid-robotics': '/docs/humanoid-robotics-course/introduction-to-humanoid-robotics',
  'sensors-and-perception': '/docs/humanoid-robotics-course/sensors-and-perception',
  'actuators-and-movement': '/docs/humanoid-robotics-course/actuators-and-movement',
  'control-systems': '/docs/humanoid-robotics-course/control-systems',
  'path-planning-and-navigation': '/docs/humanoid-robotics-course/path-planning-and-navigation',
};
```

## Entity Relationships

```
┌─────────────────┐     returns     ┌─────────────────┐
│  FubuniAgent    │───────────────▶│  AgentResponse  │
└─────────────────┘                 └────────┬────────┘
                                             │
                                             │ transforms
                                             ▼
┌─────────────────┐     sends      ┌─────────────────┐
│  ChatPage       │◀───────────────│  ChatResponse   │
│  (Frontend)     │                └─────────────────┘
└────────┬────────┘
         │
         │ uses
         ▼
┌─────────────────┐
│  ChapterMapping │
└─────────────────┘
```

## State Transitions

### Documentation Navigation State

```
┌──────────────┐
│   Initial    │ currentChapter = "introduction-to-humanoid-robotics"
└──────┬───────┘
       │
       ▼
┌──────────────┐  AI response with    ┌──────────────┐
│   Viewing    │──should_navigate────▶│  Navigating  │
│   Chapter    │                      └──────┬───────┘
└──────┬───────┘                             │
       │                                     │ iframe loads
       │ user clicks tab                     ▼
       │                              ┌──────────────┐
       └─────────────────────────────▶│   Viewing    │
                                      │ New Chapter  │
                                      └──────────────┘
```

### Mobile View State

```
┌──────────────┐
│  Chat View   │ (default on mobile)
│  (visible)   │
└──────┬───────┘
       │ toggle button
       ▼
┌──────────────┐
│   Doc View   │
│  (visible)   │
└──────┬───────┘
       │ toggle button
       ▼
┌──────────────┐
│  Chat View   │
│  (visible)   │
└──────────────┘
```
