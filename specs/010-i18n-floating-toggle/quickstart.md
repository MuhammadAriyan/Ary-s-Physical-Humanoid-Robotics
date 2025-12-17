# Quickstart: i18n with Floating Toggle & AI Translation

**Feature**: 010-i18n-floating-toggle
**Date**: 2025-12-17

## Prerequisites

- Node.js 20+
- Python 3.11
- Existing Docusaurus site running
- Better Auth configured and working
- Access to OpenAI-compatible API provider (Groq, OpenRouter, etc.)

## Quick Setup (5 minutes)

### Step 1: Update Docusaurus Config

Add Urdu locale to `docusaurus.config.ts`:

```typescript
// docusaurus.config.ts
const config: Config = {
  // ... existing config

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
};
```

### Step 2: Create i18n Directory Structure

```bash
# Create Urdu translation directories
mkdir -p i18n/ur/docusaurus-plugin-content-docs/current/humanoid-robotics-course
mkdir -p i18n/ur/docusaurus-theme-classic

# Create placeholder files
echo '{}' > i18n/ur/code.json
echo '{}' > i18n/ur/docusaurus-theme-classic/navbar.json
echo '{}' > i18n/ur/docusaurus-theme-classic/footer.json
```

### Step 3: Verify Build

```bash
# Build English (default)
npm run build

# Build Urdu locale
npm run build -- --locale ur

# Or build all locales
npm run build
```

### Step 4: Add Environment Variables

```bash
# Backend .env
OPENAI_BASE_URL=https://api.groq.com/openai/v1  # Or your provider
OPENAI_API_KEY=your-api-key
DATABASE_URL=postgresql://...  # Existing Neon connection
```

### Step 5: Run Database Migration

```bash
cd backend
python -m alembic upgrade head
# Or run SQL directly:
# psql $DATABASE_URL -f db/migrations/003_add_translations.sql
```

## Component Usage

### LanguageToggle (Auto-injected)

The floating toggle is automatically added via `Root.tsx`. No manual integration needed.

### TranslateChapter Button

Added automatically to doc pages via swizzled DocItem. Users see it if:
- They are logged in
- The page is a documentation page

## Development Commands

```bash
# Start dev server (English)
npm run start

# Start dev server (Urdu)
npm run start -- --locale ur

# Build for production
npm run build

# Test Urdu build locally
npm run serve -- --locale ur
```

## API Endpoints

| Endpoint | Method | Auth | Description |
|----------|--------|------|-------------|
| `/api/translate` | POST | Required | Trigger AI translation |
| `/api/translations` | POST | Required | Save translation |
| `/api/translations/:chapterId/:locale` | GET | None | Get saved translation |

## Testing the Feature

### Manual Testing Checklist

1. **Language Toggle**
   - [ ] Toggle appears in top-left corner
   - [ ] Clicking shows EN/UR options
   - [ ] Selecting Urdu redirects to `/ur/...` path
   - [ ] RTL layout applies to entire page
   - [ ] Selecting English returns to LTR

2. **Translation Button** (logged in)
   - [ ] "Translate to Urdu" button visible on doc pages
   - [ ] Clicking shows loading spinner
   - [ ] Translation appears after processing
   - [ ] "Save Translation" button appears
   - [ ] Saving shows success message

3. **Translation Button** (logged out)
   - [ ] Button is disabled or hidden
   - [ ] Tooltip shows "Login to translate"

4. **Saved Translations**
   - [ ] Visiting `/ur/docs/...` shows saved translation
   - [ ] If no saved translation, English content shown

## Troubleshooting

### Build fails with locale error

```bash
# Ensure i18n directory exists
ls -la i18n/ur/

# Check docusaurus.config.ts has correct locale config
grep -A 10 "i18n:" docusaurus.config.ts
```

### RTL not applying

```bash
# Check HTML dir attribute in browser dev tools
# Should be: <html dir="rtl" lang="ur">

# Verify localeConfigs.ur.direction is 'rtl'
```

### Translation API returns 401

```bash
# Verify auth token is being sent
# Check backend logs for auth middleware errors
# Ensure Better Auth session is valid
```

### Translation times out

```bash
# Check OPENAI_BASE_URL is correct
# Verify API key has quota
# Try with shorter content first
```

## File Locations Reference

| Component | Location |
|-----------|----------|
| Language Toggle | `src/components/LanguageToggle/index.tsx` |
| Translate Button | `src/components/TranslateChapter/index.tsx` |
| Translation Hook | `src/hooks/useTranslation.ts` |
| Backend Translate API | `backend/api/translate.py` |
| Backend Translations CRUD | `backend/api/translations.py` |
| DB Migration | `backend/db/migrations/003_add_translations.sql` |
| Urdu UI Strings | `i18n/ur/code.json` |
