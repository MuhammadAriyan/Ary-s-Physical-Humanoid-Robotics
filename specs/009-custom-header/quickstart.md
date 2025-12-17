# Quickstart: Custom Header/Navbar Component

**Feature Branch**: `009-custom-header`
**Date**: 2025-12-17

## Prerequisites

- Node.js 20+
- npm installed
- Project cloned and on `009-custom-header` branch

## Quick Setup

```bash
# 1. Checkout feature branch
git checkout 009-custom-header

# 2. Install dependencies
npm install

# 3. Start development server
npm start
```

The site should be running at `http://localhost:3000/Ary-s-Physical-Humanoid-Robotics/`

## File Structure

After implementation, the Header component structure will be:

```
src/
├── components/
│   └── Header/
│       ├── Header.tsx              # Main header component
│       ├── Header.module.css       # Header styles
│       ├── LanguageSelector.tsx    # Language dropdown
│       ├── UserMenu.tsx            # User avatar + dropdown
│       ├── MobileMenu.tsx          # Mobile drawer menu
│       ├── SocialLinks.tsx         # GitHub/LinkedIn icons
│       ├── icons/
│       │   ├── GitHubIcon.tsx      # GitHub SVG
│       │   ├── LinkedInIcon.tsx    # LinkedIn SVG
│       │   ├── HamburgerIcon.tsx   # Menu icon
│       │   └── CloseIcon.tsx       # Close icon
│       └── index.ts                # Exports
└── theme/
    └── Navbar/
        └── index.js                # Updated to use Header
```

## Development Workflow

### 1. Create Header Component

```tsx
// src/components/Header/Header.tsx
import React, { useState } from 'react';
import { useAuth, AuthModal } from '../Auth';
import styles from './Header.module.css';

export const Header: React.FC = () => {
  const { isAuthenticated, user, isLoading } = useAuth();
  const [showAuthModal, setShowAuthModal] = useState(false);
  const [mobileMenuOpen, setMobileMenuOpen] = useState(false);

  return (
    <header className={styles.header}>
      {/* Logo */}
      <a href="/" className={styles.logo}>
        Ary's Physical & Humanoid Robotics
      </a>

      {/* Desktop Navigation */}
      <nav className={styles.nav}>
        {/* Language selector, social links, auth button */}
      </nav>

      {/* Mobile Menu Button */}
      <button
        className={styles.hamburger}
        onClick={() => setMobileMenuOpen(true)}
        aria-label="Open menu"
      >
        {/* Hamburger icon */}
      </button>

      {/* Auth Modal */}
      <AuthModal
        isOpen={showAuthModal}
        onClose={() => setShowAuthModal(false)}
        onSuccess={() => setShowAuthModal(false)}
      />
    </header>
  );
};
```

### 2. Create Header Styles

```css
/* src/components/Header/Header.module.css */
.header {
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  height: 80px;
  background: var(--glass-white-heavy);
  backdrop-filter: var(--backdrop-blur-md) var(--backdrop-saturate);
  border-bottom: var(--border-light);
  box-shadow: var(--shadow-soft);
  z-index: var(--z-fixed);
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 0 var(--space-xl);
}

.logo {
  font-family: var(--font-display);
  font-size: var(--font-size-xl);
  font-weight: var(--font-weight-semibold);
  color: var(--text-primary);
  text-decoration: none;
}

/* Dark mode */
[data-theme='dark'] .header {
  background: var(--glass-black-heavy);
  border-bottom-color: rgba(255, 255, 255, 0.1);
}

/* Mobile */
@media (max-width: 768px) {
  .header {
    height: 70px;
    padding: 0 var(--space-md);
  }
}
```

### 3. Integrate with Docusaurus

```jsx
// src/theme/Navbar/index.js
import React from 'react';
import { Header } from '@site/src/components/Header';

export default function NavbarWrapper() {
  return <Header />;
}
```

## Testing

### Manual Testing Checklist

- [ ] Logo displays correctly and links to homepage
- [ ] Auth button shows "Sign In" when logged out
- [ ] Auth button shows avatar when logged in
- [ ] AuthModal opens on "Sign In" click
- [ ] User dropdown shows on avatar click
- [ ] Sign out works correctly
- [ ] GitHub icon links to correct URL
- [ ] LinkedIn icon links to correct URL
- [ ] Language selector shows current locale
- [ ] Mobile hamburger appears at < 768px
- [ ] Mobile menu opens/closes correctly
- [ ] Dark mode styles apply correctly
- [ ] Keyboard navigation works (Tab, Enter, Escape)

### Viewport Testing

- [ ] Desktop (1920px)
- [ ] Laptop (1280px)
- [ ] Tablet (768px)
- [ ] Mobile (375px)

## Troubleshooting

### Header not showing
- Check `src/theme/Navbar/index.js` imports Header correctly
- Verify AuthProvider wraps the app in `src/theme/Root.tsx`

### Styles not applying
- Ensure CSS Modules import is correct: `import styles from './Header.module.css'`
- Check CSS variables are available (imported in custom.css)

### Auth not working
- Verify auth service URL in `src/lib/auth-client.ts`
- Check browser console for CORS errors
- Ensure cookies are being set (check Application tab)

### Mobile menu not appearing
- Check viewport width and media query breakpoint
- Verify hamburger button has correct onClick handler
- Check z-index values for menu overlay
