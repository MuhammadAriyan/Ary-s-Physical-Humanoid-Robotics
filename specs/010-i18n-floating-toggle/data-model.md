# Data Model: i18n with Floating Toggle & AI Translation

**Feature**: 010-i18n-floating-toggle
**Date**: 2025-12-17

## Entity Relationship Diagram

```
┌─────────────────┐       ┌─────────────────────┐       ┌─────────────┐
│     User        │       │ ChapterTranslation  │       │   Locale    │
├─────────────────┤       ├─────────────────────┤       ├─────────────┤
│ id (PK)         │──┐    │ id (PK)             │       │ code (PK)   │
│ email           │  │    │ chapter_id          │───┐   │ label       │
│ name            │  └───►│ translated_by (FK)  │   │   │ direction   │
│ ...             │       │ locale (FK)         │◄──┼───│ html_lang   │
└─────────────────┘       │ content             │   │   └─────────────┘
                          │ created_at          │   │
                          │ updated_at          │   │   ┌─────────────┐
                          └─────────────────────┘   │   │   Chapter   │
                                                    │   ├─────────────┤
                                                    └───│ id (PK)     │
                                                        │ path        │
                                                        │ title       │
                                                        └─────────────┘
```

## Entities

### 1. Locale (Static Configuration)

**Purpose**: Represents supported languages with display properties.

**Storage**: Defined in `docusaurus.config.ts` (not database)

| Field | Type | Description | Example |
|-------|------|-------------|---------|
| code | string (PK) | ISO language code | `en`, `ur` |
| label | string | Display name in native script | `English`, `اردو` |
| direction | enum | Text direction | `ltr`, `rtl` |
| htmlLang | string | HTML lang attribute value | `en`, `ur` |

**Values**:
```typescript
const locales = {
  en: { label: 'English', direction: 'ltr', htmlLang: 'en' },
  ur: { label: 'اردو', direction: 'rtl', htmlLang: 'ur' }
};
```

---

### 2. Chapter (Virtual Entity)

**Purpose**: Represents a documentation page that can be translated.

**Storage**: Derived from file system path (not database)

| Field | Type | Description | Example |
|-------|------|-------------|---------|
| id | string (PK) | Unique identifier derived from path | `humanoid-robotics-course/intro` |
| path | string | URL path to the chapter | `/docs/humanoid-robotics-course/intro` |
| title | string | Chapter title from frontmatter | `Introduction to Humanoid Robotics` |
| content | string | Markdown content | `# Introduction...` |

**Derivation Logic**:
```typescript
// Chapter ID is derived from the docs path
const chapterId = location.pathname
  .replace(/^\/docs\//, '')  // Remove /docs/ prefix
  .replace(/\/$/, '');       // Remove trailing slash
// Result: "humanoid-robotics-course/introduction-to-humanoid-robotics"
```

---

### 3. ChapterTranslation (Database Entity)

**Purpose**: Stores AI-generated translations that have been saved by users.

**Storage**: PostgreSQL table `chapter_translations`

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | SERIAL | PRIMARY KEY | Auto-increment identifier |
| chapter_id | VARCHAR(255) | NOT NULL | Reference to chapter path |
| locale | VARCHAR(10) | NOT NULL | Target language code (e.g., `ur`) |
| content | TEXT | NOT NULL | Translated markdown content |
| translated_by | INTEGER | FOREIGN KEY → users.id | User who triggered translation |
| created_at | TIMESTAMP | DEFAULT NOW() | When translation was created |
| updated_at | TIMESTAMP | DEFAULT NOW() | When translation was last updated |

**Constraints**:
- `UNIQUE(chapter_id, locale)` - One translation per chapter per locale

**SQL Schema**:
```sql
CREATE TABLE chapter_translations (
  id SERIAL PRIMARY KEY,
  chapter_id VARCHAR(255) NOT NULL,
  locale VARCHAR(10) NOT NULL DEFAULT 'ur',
  content TEXT NOT NULL,
  translated_by INTEGER REFERENCES users(id) ON DELETE SET NULL,
  created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,

  CONSTRAINT unique_chapter_locale UNIQUE (chapter_id, locale)
);

-- Index for fast lookups by chapter
CREATE INDEX idx_chapter_translations_chapter_id ON chapter_translations(chapter_id);

-- Index for finding user's translations
CREATE INDEX idx_chapter_translations_user ON chapter_translations(translated_by);
```

---

### 4. User (Existing Entity - Extended Reference)

**Purpose**: Existing user entity from Better Auth system.

**Storage**: PostgreSQL table `users` (existing)

| Field | Type | Description |
|-------|------|-------------|
| id | INTEGER (PK) | User identifier |
| email | VARCHAR | User email |
| name | VARCHAR | Display name |
| ... | ... | Other Better Auth fields |

**Relationship**: `ChapterTranslation.translated_by` → `User.id`

---

### 5. UserPreference (Browser Storage)

**Purpose**: Stores user's language preference locally.

**Storage**: localStorage

| Key | Type | Description | Example |
|-----|------|-------------|---------|
| `docusaurus.locale` | string | Preferred locale code | `ur` |

**Operations**:
```typescript
// Read preference
const preferredLocale = localStorage.getItem('docusaurus.locale') || 'en';

// Write preference (on language switch)
localStorage.setItem('docusaurus.locale', 'ur');
```

---

## State Transitions

### Translation Status State Machine

```
                          ┌─────────────┐
                          │ Untranslated│ (no saved translation exists)
                          └──────┬──────┘
                                 │
                    User clicks "Translate"
                                 │
                                 ▼
                          ┌─────────────┐
                          │ Translating │ (API call in progress)
                          └──────┬──────┘
                                 │
                    ┌────────────┼────────────┐
                    │            │            │
                Success       Error       Cancel
                    │            │            │
                    ▼            ▼            ▼
            ┌───────────┐ ┌───────────┐ ┌───────────┐
            │ Translated│ │  Failed   │ │Untranslated│
            │ (unsaved) │ │           │ │           │
            └─────┬─────┘ └─────┬─────┘ └───────────┘
                  │             │
        User clicks │      User clicks
        "Save"      │      "Retry"
                  │             │
                  ▼             │
            ┌───────────┐       │
            │   Saved   │◄──────┘
            │           │
            └───────────┘
```

### TypeScript State Type

```typescript
type TranslationStatus =
  | { state: 'untranslated' }
  | { state: 'translating'; startedAt: Date }
  | { state: 'translated'; content: string; savedToDb: false }
  | { state: 'saved'; content: string; translationId: number }
  | { state: 'failed'; error: string; canRetry: boolean };
```

---

## Validation Rules

### ChapterTranslation

| Field | Rule | Error Message |
|-------|------|---------------|
| chapter_id | Required, max 255 chars | "Chapter ID is required" |
| chapter_id | Must match valid path pattern | "Invalid chapter path" |
| locale | Must be in ['ur'] | "Unsupported locale" |
| content | Required, non-empty | "Translation content is required" |
| translated_by | Must be valid user ID | "Invalid user" |

### Path Pattern Validation

```typescript
const CHAPTER_ID_PATTERN = /^[a-z0-9-]+\/[a-z0-9-]+$/;

function isValidChapterId(id: string): boolean {
  return CHAPTER_ID_PATTERN.test(id) && id.length <= 255;
}
```

---

## API Contracts

### Translate Chapter

**Endpoint**: `POST /api/translate`

**Request**:
```typescript
interface TranslateRequest {
  chapterId: string;      // e.g., "humanoid-robotics-course/intro"
  content: string;        // Original markdown content
  targetLocale: 'ur';     // Target language
}
```

**Response**:
```typescript
interface TranslateResponse {
  success: boolean;
  translation?: string;   // Translated markdown
  error?: string;         // Error message if failed
}
```

### Save Translation

**Endpoint**: `POST /api/translations`

**Request**:
```typescript
interface SaveTranslationRequest {
  chapterId: string;
  locale: 'ur';
  content: string;        // Translated content to save
}
```

**Response**:
```typescript
interface SaveTranslationResponse {
  success: boolean;
  translationId?: number; // ID of saved translation
  error?: string;
}
```

### Get Saved Translation

**Endpoint**: `GET /api/translations/:chapterId/:locale`

**Response**:
```typescript
interface GetTranslationResponse {
  exists: boolean;
  translation?: {
    id: number;
    content: string;
    translatedBy: string;  // User name
    updatedAt: string;     // ISO timestamp
  };
}
```

---

## Data Flow

### 1. Loading a Chapter Page

```
User navigates to /docs/chapter-name
           │
           ▼
┌─────────────────────────────┐
│ Check URL locale prefix     │
│ (none = 'en', /ur = 'ur')   │
└──────────────┬──────────────┘
               │
               ▼
┌─────────────────────────────┐
│ Load chapter content        │
│ (from filesystem/build)     │
└──────────────┬──────────────┘
               │
     ┌─────────┴─────────┐
     │                   │
  If 'ur'             If 'en'
     │                   │
     ▼                   ▼
┌─────────────┐    ┌─────────────┐
│ Check DB    │    │ Show English│
│ for saved   │    │ + Translate │
│ translation │    │ button      │
└──────┬──────┘    └─────────────┘
       │
   ┌───┴───┐
   │       │
 Exists   None
   │       │
   ▼       ▼
┌──────┐ ┌──────────┐
│Show  │ │Show EN   │
│Saved │ │fallback  │
│Urdu  │ │+ Translate│
└──────┘ └──────────┘
```

### 2. Translating a Chapter

```
User clicks "Translate to Urdu"
           │
           ▼
┌─────────────────────────────┐
│ Check authentication        │
└──────────────┬──────────────┘
               │
     ┌─────────┴─────────┐
     │                   │
  Logged in         Not logged in
     │                   │
     ▼                   ▼
┌─────────────┐    ┌─────────────┐
│ Show loader │    │ Show login  │
│ POST /api/  │    │ prompt      │
│ translate   │    └─────────────┘
└──────┬──────┘
       │
       ▼
┌─────────────────────────────┐
│ Backend calls AI provider   │
│ (via OPENAI_BASE_URL)       │
└──────────────┬──────────────┘
               │
     ┌─────────┴─────────┐
     │                   │
  Success             Error
     │                   │
     ▼                   ▼
┌─────────────┐    ┌─────────────┐
│ Display     │    │ Show error  │
│ translation │    │ + retry btn │
│ + Save btn  │    └─────────────┘
└─────────────┘
```
