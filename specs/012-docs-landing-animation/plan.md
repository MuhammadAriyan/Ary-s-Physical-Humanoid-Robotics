# Implementation Plan: Custom /docs Landing Page with Animated Intro

**Branch**: `012-docs-landing-animation` | **Date**: 2025-12-25 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification for animated docs landing page

## Summary

Create a custom Docusaurus docs landing page with a 5-phase animated intro sequence that:
1. Fades in "Journey into" text (0-0.8s)
2. Fades in "Ary's Humanoid Robots and Physical AI" (0.8-1.6s)
3. Shows a swirling arrow (1.6-2.5s)
4. Fades out intro while arrow decelerates (2.5-3.2s)
5. Reveals book chapters navigation (3.2s+)

Uses existing design system (Inter, Playfair Display, CSS variables) and maintains black/white aesthetic.

## Technical Context

**Language/Version**: TypeScript 5.6 (React 19, Docusaurus 3.9.2)
**Primary Dependencies**: React 19, Docusaurus classic preset, CSS custom properties
**Storage**: N/A (static site, no backend)
**Testing**: Manual visual testing, Lighthouse accessibility audit
**Target Platform**: GitHub Pages (static HTML/CSS/JS)
**Project Type**: Docusaurus documentation page
**Performance Goals**: 60fps animations, <2s page load
**Constraints**: GPU-accelerated animations only, respect prefers-reduced-motion
**Scale/Scope**: Single landing page component + CSS styles

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Verify compliance with Fubuni Docs Agent Constitution:

| Requirement | Compliance | Notes |
|-------------|------------|-------|
| Agent MUST be named "Fubuni" | N/A | Not applicable - frontend feature |
| Only OpenAI-compatible providers | N/A | No backend/API calls |
| Backend Python 3.11 + OpenAI Agents SDK | N/A | No backend required |
| Frontend matches Docusaurus theme | ✅ PASS | Uses existing CSS variables |
| Fully static deployable | ✅ PASS | GitHub Pages deployment |
| No vector DB, crawling, or embeddings | ✅ PASS | Pure frontend animations |
| Architecture supports later ingestion | ✅ PASS | Markdown content structure unchanged |

**Result**: ✅ PASS - This is a frontend-only documentation feature that aligns with static deployability goals.

## Project Structure

### Documentation (this feature)

```text
specs/012-docs-landing-animation/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (skipped - requirements clear)
├── data-model.md        # Phase 1 output (skipped - no data entities)
├── quickstart.md        # Phase 1 output (CONTRIBUTING guide)
├── contracts/           # Phase 1 output (skipped - no APIs)
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

```text
src/
├── pages/
│   └── docs.tsx              # Custom docs landing page (NEW)
└── css/
    └── docs-landing.css      # Animation styles (NEW)
```

**Structure Decision**: This is a documentation enhancement feature. We create:
- `src/pages/docs.tsx` - React component for the custom landing page
- `src/css/docs-landing.css` - Keyframe animations and component styles

Existing files modified:
- `docusaurus.config.ts` - Optional route configuration

## Phase 0: Research (SKIPPED)

Research phase is not required for this feature because:
- All requirements are clearly defined in the specification
- Animation patterns are already established in `src/styles/animations.css`
- Design system variables are already defined in `src/css/custom.css`
- No technical unknowns requiring investigation

## Phase 1: Design

### Directory Structure

Create the following files:

```bash
# Create landing page component
touch src/pages/docs.tsx

# Create animation styles
touch src/css/docs-landing.css
```

### Animation Keyframes

Add to `src/css/docs-landing.css`:

```css
@keyframes textFadeIn {
  from { opacity: 0; transform: translateY(20px); }
  to { opacity: 1; transform: translateY(0); }
}

@keyframes arrowSwirl {
  0% { transform: translateX(0) rotate(0deg); }
  25% { transform: translateX(30px) rotate(90deg); }
  50% { transform: translateX(-20px) rotate(180deg); }
  75% { transform: translateX(15px) rotate(270deg); }
  100% { transform: translateX(0) rotate(360deg); }
}

@keyframes textFadeOut {
  to { opacity: 0; transform: scale(0.95); }
}

@keyframes chaptersFadeIn {
  from { opacity: 0; }
  to { opacity: 1; }
}
```

### Landing Page Component Structure

```typescript
// src/pages/docs.tsx
export default function DocsLanding() {
  const [phase, setPhase] = useState('intro');

  useEffect(() => {
    const timers = [
      setTimeout(() => setPhase('title'), 800),
      setTimeout(() => setPhase('arrow'), 1600),
      setTimeout(() => setPhase('exit'), 2500),
      setTimeout(() => setPhase('chapters'), 3200),
    ];
    return () => timers.forEach(clearTimeout);
  }, []);

  return (
    <div className="docs-landing">
      <div className={`intro ${phase !== 'chapters' ? 'visible' : ''}`}>
        <h1 className="journey-text">Journey into</h1>
        <h1 className="title-text">Ary's Humanoid Robots and Physical AI</h1>
        <div className="arrow-container">
          <svg className="arrow">...</svg>
        </div>
      </div>
      <div className={`chapters ${phase === 'chapters' ? 'visible' : ''}`}>
        {/* Chapter navigation from sidebars.ts */}
      </div>
    </div>
  );
}
```

### Quickstart Guide (CONTRIBUTING.md)

```markdown
# Docs Landing Page Animation

## Animation Sequence

1. "Journey into" fades in (0-0.8s)
2. Title text fades in (0.8-1.6s)
3. Arrow appears and swirls (1.6-2.5s)
4. Intro fades out (2.5-3.2s)
5. Chapters appear (3.2s+)

## Modifying Timings

Edit `src/pages/docs.tsx` timeout values to adjust timing:

```javascript
setTimeout(() => setPhase('title'), 800),  // First text delay
setTimeout(() => setPhase('arrow'), 1600), // Arrow delay
setTimeout(() => setPhase('exit'), 2500),  // Exit delay
setTimeout(() => setPhase('chapters'), 3200) // Chapters delay
```

## Accessibility

The page respects `prefers-reduced-motion`. Users with motion sensitivity see immediate chapter navigation.
```

## Complexity Tracking

*Not applicable - no Constitution violations.*

## Phase 2: Implementation Planning

### Key Implementation Tasks

1. **Create `src/pages/docs.tsx`**
   - React component with animation state machine
   - Render intro text with fade-in animations
   - Render swirling arrow SVG
   - Render chapter navigation from sidebars.ts

2. **Create `src/css/docs-landing.css`**
   - Keyframe animations (textFadeIn, arrowSwirl, textFadeOut, chaptersFadeIn)
   - Component styling using CSS variables
   - Dark mode support with media queries
   - Reduced motion support

3. **Configure Docusaurus route (optional)**
   - Update `routeBasePath` if needed
   - Or use `index_route` in docs plugin config

4. **Testing**
   - Verify animation sequence timing
   - Test dark mode colors
   - Test reduced motion preference
   - Lighthouse accessibility audit

## Next Steps

After this plan:
1. Run `/sp.tasks` to generate detailed implementation tasks
2. Execute tasks to create landing page component and styles
3. Test animation sequence and accessibility
4. Build and deploy to GitHub Pages
