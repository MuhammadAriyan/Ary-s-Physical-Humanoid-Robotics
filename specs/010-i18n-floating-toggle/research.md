# Research: Docusaurus i18n with Floating Toggle & AI Translation

**Feature**: 010-i18n-floating-toggle
**Date**: 2025-12-17
**Status**: Complete

## Research Questions & Findings

### 1. Docusaurus i18n Configuration

**Question**: How to configure i18n for English/Urdu with RTL support?

**Decision**: Use Docusaurus native i18n with `localeConfigs` for RTL direction

**Rationale**: Docusaurus 3.x has built-in i18n support that automatically handles URL routing, locale detection, and direction attributes. Using the native system ensures compatibility with future updates.

**Implementation**:
```typescript
// docusaurus.config.ts
i18n: {
  defaultLocale: 'en',
  locales: ['en', 'ur'],
  localeConfigs: {
    en: {
      label: 'English',
      direction: 'ltr',
      htmlLang: 'en',
    },
    ur: {
      label: 'اردو',
      direction: 'rtl',
      htmlLang: 'ur',
    },
  },
},
```

**Alternatives Considered**:
- Custom i18n solution: Rejected - reinvents the wheel, loses Docusaurus features
- Third-party i18n library (react-i18next): Rejected - adds complexity, Docusaurus already provides this

**Sources**:
- [Docusaurus i18n Tutorial](https://docusaurus.io/docs/i18n/tutorial)
- [Docusaurus Client API](https://docusaurus.io/docs/docusaurus-core)

---

### 2. Custom Language Switcher (Not Navbar)

**Question**: How to create a floating language toggle instead of using navbar dropdown?

**Decision**: Create a custom React component using `useDocusaurusContext` hook for locale data and `useLocation` for current path

**Rationale**:
- FR-010 requires NOT using navbar for language toggle
- Docusaurus provides hooks to access i18n context (`useDocusaurusContext`)
- Custom component can be injected via `Root.tsx` (existing pattern in codebase)

**Implementation Approach**:
```typescript
// src/theme/LanguageToggle/index.tsx
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { useLocation } from '@docusaurus/router';

function LanguageToggle() {
  const { i18n: { currentLocale, locales, localeConfigs } } = useDocusaurusContext();
  const location = useLocation();

  // Build alternate locale URLs
  const getLocalePath = (locale: string) => {
    const pathWithoutLocale = location.pathname.replace(/^\/(en|ur)/, '');
    return locale === 'en' ? pathWithoutLocale || '/' : `/${locale}${pathWithoutLocale}`;
  };

  // ... render floating toggle
}
```

**Alternatives Considered**:
- `localeDropdown` navbar item: Rejected - FR-010 explicitly prohibits navbar placement
- Swizzle entire navbar: Rejected - overkill, only need language toggle

---

### 3. URL-Based Language Routing

**Question**: How does Docusaurus handle URL routing for locales?

**Decision**: Use default Docusaurus URL pattern - `/ur/docs/...` for Urdu, no prefix for English (default)

**Rationale**:
- Docusaurus automatically adds `/<locale>/` prefix for non-default locales
- This enables deep-linking to translated content
- Persists language preference via URL (works without JavaScript/localStorage)

**URL Patterns**:
- English (default): `https://site.com/docs/intro`
- Urdu: `https://site.com/ur/docs/intro`

**Alternatives Considered**:
- Query parameter (`?lang=ur`): Rejected - not SEO-friendly, not Docusaurus standard
- Subdomain (`ur.site.com`): Rejected - requires DNS setup, complicates deployment

---

### 4. RTL Layout Support

**Question**: How to apply RTL layout when Urdu is selected?

**Decision**: Rely on Docusaurus automatic `dir="rtl"` attribute + CSS logical properties

**Rationale**:
- Docusaurus automatically sets `dir` attribute on HTML based on `localeConfigs.direction`
- CSS logical properties (`margin-inline-start`, `padding-inline-end`) automatically flip for RTL
- Existing codebase uses `[data-theme='dark']` pattern - can add `[dir='rtl']` similarly

**Implementation**:
```css
/* Base styles use logical properties */
.component {
  margin-inline-start: 1rem; /* Left in LTR, Right in RTL */
  padding-inline-end: 0.5rem;
}

/* RTL-specific overrides if needed */
[dir='rtl'] .component {
  /* RTL-specific adjustments */
}
```

**Existing CSS Variables** (from codebase exploration):
- `--ifm-*` Infima variables already support RTL
- Custom glassmorphism styles may need RTL-specific tweaks

**Alternatives Considered**:
- Manual `dir` attribute management: Rejected - Docusaurus handles this automatically
- Separate RTL stylesheet: Rejected - CSS logical properties are cleaner

---

### 5. Floating Toggle Position & Styling

**Question**: Where to position the floating toggle and how to style it?

**Decision**: Position at **top-left corner** (`position: fixed; top: 20px; left: 20px`) with glassmorphism styling

**Rationale**:
- Top-left avoids collision with existing components:
  - FubuniChat bubble: bottom-right (20px, 20px)
  - AuthButton: top-right (1rem, 1rem)
- Glassmorphism matches existing design system (`--glass-*` variables)
- Uses globe icon for universal language recognition

**Implementation Pattern** (from FubuniBubble.tsx):
```css
.languageToggle {
  position: fixed;
  top: 20px;
  left: 20px;
  width: 44px; /* Minimum tap target per SC-008 */
  height: 44px;
  border-radius: 50%;
  z-index: 9998; /* Below chat bubble (9999) */
  backdrop-filter: blur(10px);
  background: var(--glass-bg-light);
  box-shadow: var(--glass-shadow-light);
  transition: all var(--transition-smooth);
}

[data-theme='dark'] .languageToggle {
  background: var(--glass-bg-dark);
}
```

**Mobile Considerations**:
- Same position works on mobile
- 44x44px meets accessibility tap target requirements
- Dropdown expands downward, doesn't overlap navbar

---

### 6. Translation File Structure

**Question**: How to organize translation files for Urdu?

**Decision**: Use Docusaurus standard i18n folder structure

**Rationale**: Following Docusaurus conventions ensures compatibility with build process and allows incremental translation.

**Structure**:
```
i18n/
└── ur/
    ├── code.json                           # UI string translations
    ├── docusaurus-plugin-content-docs/
    │   └── current/                        # Translated doc files
    │       └── humanoid-robotics-course/
    │           └── *.md                    # Urdu translations
    ├── docusaurus-plugin-content-blog/     # If blog exists
    ├── docusaurus-plugin-content-pages/    # Custom pages
    └── docusaurus-theme-classic/
        ├── navbar.json                     # Navbar labels
        └── footer.json                     # Footer labels
```

**Initial State**: Empty/placeholder files - build succeeds per FR-011

---

### 7. AI Translation Integration

**Question**: How to implement AI-powered chapter translation for logged-in users?

**Decision**: Backend API endpoint using OpenAI-compatible provider (per constitution), frontend React component with auth check

**Rationale**:
- Constitution mandates OpenAI-compatible providers via OPENAI_BASE_URL
- Existing auth system (Better Auth) can gate the translate button
- Backend handles API call to avoid exposing API keys in frontend

**Architecture**:
```
[Frontend]                    [Backend]                    [AI Provider]
TranslateButton ─────────────► POST /api/translate ───────► Groq/OpenRouter
     │                              │                            │
     │ (auth token)                 │ (OPENAI_BASE_URL)          │
     │                              │                            │
     ◄──────────────────────────────◄───────────────────────────◄
           Urdu translation                 Response
```

**Frontend Component**:
```typescript
// src/components/TranslateChapter/index.tsx
import { useAuth } from '../Auth';

function TranslateChapter({ chapterContent, chapterId }) {
  const { isAuthenticated, user } = useAuth();
  const [translating, setTranslating] = useState(false);
  const [translatedContent, setTranslatedContent] = useState(null);

  if (!isAuthenticated) {
    return <Tooltip content="Login to translate">
      <Button disabled>Translate to Urdu</Button>
    </Tooltip>;
  }

  const handleTranslate = async () => {
    setTranslating(true);
    const response = await fetch('/api/translate', {
      method: 'POST',
      headers: { 'Authorization': `Bearer ${user.token}` },
      body: JSON.stringify({ content: chapterContent, targetLang: 'ur' })
    });
    const { translation } = await response.json();
    setTranslatedContent(translation);
    setTranslating(false);
  };

  // ... render button and translated content
}
```

**Backend Endpoint** (FastAPI):
```python
# backend/api/translate.py
from openai import OpenAI

client = OpenAI(base_url=os.getenv("OPENAI_BASE_URL"))

@router.post("/api/translate")
async def translate(request: TranslateRequest, user: User = Depends(get_current_user)):
    response = client.chat.completions.create(
        model="llama-3.1-70b-versatile",  # or configured model
        messages=[
            {"role": "system", "content": "Translate to Urdu. Preserve markdown formatting."},
            {"role": "user", "content": request.content}
        ]
    )
    return {"translation": response.choices[0].message.content}
```

**Alternatives Considered**:
- Client-side API call: Rejected - exposes API keys
- Pre-translate all content: Rejected - expensive, most content may never be read in Urdu
- Google Translate API: Rejected - not OpenAI-compatible per constitution

---

### 8. Saving Translations

**Question**: How to persist AI-generated translations?

**Decision**: Store in database (Neon PostgreSQL) with chapter ID and locale

**Rationale**:
- Existing Neon database infrastructure from Better Auth feature
- Enables fast retrieval of previously translated content
- Avoids redundant API calls

**Data Model**:
```sql
CREATE TABLE chapter_translations (
  id SERIAL PRIMARY KEY,
  chapter_id VARCHAR(255) NOT NULL,  -- e.g., "humanoid-robotics-course/intro"
  locale VARCHAR(10) NOT NULL,       -- "ur"
  content TEXT NOT NULL,             -- Translated markdown
  translated_by INTEGER REFERENCES users(id),
  created_at TIMESTAMP DEFAULT NOW(),
  updated_at TIMESTAMP DEFAULT NOW(),
  UNIQUE(chapter_id, locale)
);
```

**Alternatives Considered**:
- Write to i18n files directly: Rejected - requires rebuild, not dynamic
- localStorage: Rejected - not shared across users
- Separate translation service: Rejected - over-engineering for MVP

---

### 9. Swizzling Strategy

**Question**: Which Docusaurus components need swizzling?

**Decision**: Minimal swizzling - inject via existing Root.tsx pattern

**Components to Create/Modify**:
1. `src/theme/Root.tsx` - Add LanguageToggle to existing providers
2. `src/components/LanguageToggle/` - New floating toggle component
3. `src/components/TranslateChapter/` - New translate button component
4. Swizzle `DocItem` wrapper to inject TranslateChapter button

**Swizzle Commands**:
```bash
# Wrap DocItem to add translate button at top of each doc
npm run swizzle @docusaurus/theme-classic DocItem/Content -- --wrap
```

**Alternatives Considered**:
- Swizzle entire theme: Rejected - too invasive, hard to maintain
- MDX component injection: Rejected - requires modifying every doc file

---

### 10. Persistence Strategy

**Question**: How to persist language preference across sessions?

**Decision**: Dual persistence - URL path (primary) + localStorage (secondary)

**Rationale**:
- URL path: Works without JavaScript, SEO-friendly, shareable
- localStorage: Enables redirect to preferred locale on homepage visit

**Implementation**:
```typescript
// On site load (in Root.tsx or useEffect)
const storedLocale = localStorage.getItem('docusaurus.locale');
const urlLocale = location.pathname.startsWith('/ur') ? 'ur' : 'en';

// If on homepage and has stored preference different from current
if (location.pathname === '/' && storedLocale && storedLocale !== urlLocale) {
  navigate(`/${storedLocale}/`);
}

// On language switch
localStorage.setItem('docusaurus.locale', newLocale);
navigate(getLocalePath(newLocale));
```

**Alternatives Considered**:
- Cookie-based: Rejected - requires server-side handling
- User account preference: Out of scope - uses browser-local only

---

## Summary of Decisions

| Area | Decision | Key Reason |
|------|----------|------------|
| i18n Config | Native Docusaurus i18n | Built-in support, automatic RTL |
| Language Toggle | Custom floating component | FR-010 prohibits navbar |
| Position | Top-left (20px, 20px) | Avoids existing components |
| URL Pattern | `/ur/docs/...` | Docusaurus default, SEO-friendly |
| RTL Support | CSS logical properties | Automatic flip, maintainable |
| AI Translation | Backend API + OpenAI-compatible | Constitution compliance |
| Storage | Neon PostgreSQL | Existing infrastructure |
| Persistence | URL + localStorage | Works with/without JS |

## Open Questions / Future Considerations

1. **Rate Limiting**: Should we limit translations per user per day?
2. **Translation Quality**: Should we add a feedback mechanism for poor translations?
3. **Caching**: Should we cache translations in Redis for faster retrieval?
4. **Analytics**: Track which chapters are most frequently translated?
