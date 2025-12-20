# Quickstart: Chat Page with Documentation Integration

**Feature**: 009-chat-page-chatkit
**Date**: 2025-12-20

## Prerequisites

- Node.js 20+
- Python 3.11+
- Existing Fubuni backend running
- Docusaurus dev server or build

## Quick Setup

### 1. Install Frontend Dependencies

```bash
cd /home/ary/Dev/abc/Ary-s-Physical-Humanoid-Robotics
npm install @openai/chat-kit-react
```

### 2. Backend Changes

**File**: `backend/app/agents/fubuni_agent.py`

Add structured output model:

```python
from pydantic import BaseModel
from typing import Optional

class AgentResponse(BaseModel):
    response: str
    chapter: Optional[str] = None
    section: Optional[str] = None
    should_navigate: bool = False
```

Update Agent with output_type:

```python
self.agent = Agent(
    name="Fubuni",
    instructions=INSTRUCTIONS,  # Updated with chapter mapping
    tools=[search_knowledge_base, ...],
    output_type=AgentResponse
)
```

### 3. Create Chat Page

**File**: `src/pages/chat.tsx`

```tsx
import React, { useState } from 'react';
import Layout from '@theme/Layout';
import BrowserOnly from '@docusaurus/BrowserOnly';
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from './chat.module.css';

const DOC_CHAPTERS = {
  'introduction-to-humanoid-robotics': '/docs/humanoid-robotics-course/introduction-to-humanoid-robotics',
  'sensors-and-perception': '/docs/humanoid-robotics-course/sensors-and-perception',
  'actuators-and-movement': '/docs/humanoid-robotics-course/actuators-and-movement',
  'control-systems': '/docs/humanoid-robotics-course/control-systems',
  'path-planning-and-navigation': '/docs/humanoid-robotics-course/path-planning-and-navigation',
};

export default function ChatPage() {
  return (
    <Layout title="Chat with Fubuni">
      <BrowserOnly fallback={<div>Loading...</div>}>
        {() => <ChatPageContent />}
      </BrowserOnly>
    </Layout>
  );
}

function ChatPageContent() {
  const [currentChapter, setCurrentChapter] = useState('introduction-to-humanoid-robotics');
  const [isMobileDocVisible, setIsMobileDocVisible] = useState(false);
  const baseUrl = useBaseUrl('/');

  const handleAgentResponse = (response: any) => {
    if (response.chapter && response.should_navigate) {
      setCurrentChapter(response.chapter);
    }
  };

  return (
    <div className={styles.container}>
      <div className={`${styles.docViewer} ${isMobileDocVisible ? styles.visible : ''}`}>
        <div className={styles.chapterTabs}>
          {Object.keys(DOC_CHAPTERS).map((chapter) => (
            <button
              key={chapter}
              className={`${styles.tab} ${currentChapter === chapter ? styles.active : ''}`}
              onClick={() => setCurrentChapter(chapter)}
            >
              {chapter.replace(/-/g, ' ')}
            </button>
          ))}
        </div>
        <iframe
          src={`${baseUrl}${DOC_CHAPTERS[currentChapter]}`}
          className={styles.iframe}
          title="Documentation"
        />
      </div>
      <div className={styles.chatPanel}>
        {/* ChatKit or custom chat component */}
        <button
          className={styles.mobileToggle}
          onClick={() => setIsMobileDocVisible(!isMobileDocVisible)}
        >
          {isMobileDocVisible ? 'Show Chat' : 'Show Docs'}
        </button>
        {/* Chat interface here */}
      </div>
    </div>
  );
}
```

### 4. Create Styles

**File**: `src/pages/chat.module.css`

```css
.container {
  display: grid;
  grid-template-columns: 1fr 400px;
  height: calc(100vh - 60px);
}

.docViewer {
  display: flex;
  flex-direction: column;
  border-right: 1px solid var(--ifm-color-emphasis-300);
}

.chapterTabs {
  display: flex;
  gap: 0.5rem;
  padding: 0.5rem;
  overflow-x: auto;
  background: var(--ifm-background-surface-color);
  border-bottom: 1px solid var(--ifm-color-emphasis-300);
}

.tab {
  padding: 0.5rem 1rem;
  border: none;
  background: transparent;
  cursor: pointer;
  white-space: nowrap;
  text-transform: capitalize;
  font-size: 0.875rem;
}

.tab.active {
  background: var(--ifm-color-primary);
  color: white;
  border-radius: 4px;
}

.iframe {
  flex: 1;
  width: 100%;
  border: none;
}

.chatPanel {
  display: flex;
  flex-direction: column;
  background: var(--ifm-background-color);
}

.mobileToggle {
  display: none;
}

@media (max-width: 768px) {
  .container {
    grid-template-columns: 1fr;
  }

  .docViewer {
    display: none;
    position: fixed;
    inset: 60px 0 0 0;
    z-index: 100;
    background: var(--ifm-background-color);
  }

  .docViewer.visible {
    display: flex;
  }

  .mobileToggle {
    display: block;
    padding: 0.75rem;
    background: var(--ifm-color-primary);
    color: white;
    border: none;
    cursor: pointer;
  }
}
```

## Testing

### Local Development

```bash
# Terminal 1: Backend
cd backend && python -m uvicorn app.main:app --reload

# Terminal 2: Frontend
npm start
```

Visit `http://localhost:3000/chat`

### Test Cases

1. **Split View**: Page shows docs on left, chat on right
2. **Auto Navigation**: Ask "tell me about sensors" → docs navigate to sensors chapter
3. **Manual Navigation**: Click chapter tabs → docs update
4. **Mobile Toggle**: Resize to < 768px → single panel with toggle

## Deployment

### Frontend (GitHub Pages)

```bash
npm run build
# Deploy via GitHub Actions
```

### Backend (Hugging Face Spaces)

```bash
cd backend
# Use huggingface_hub to upload
```

## Files Changed

| File | Action | Description |
|------|--------|-------------|
| `backend/app/agents/fubuni_agent.py` | MODIFY | Add AgentResponse, output_type |
| `backend/app/api/chat.py` | MODIFY | Update ChatResponse |
| `src/pages/chat.tsx` | CREATE | New chat page |
| `src/pages/chat.module.css` | CREATE | Page styling |
| `package.json` | MODIFY | Add @openai/chat-kit-react |
