/**
 * Component Interface Contracts: Custom Header/Navbar
 *
 * These interfaces define the contracts between Header sub-components.
 * Implementation MUST adhere to these interfaces.
 */

import { ReactNode } from 'react';

// ============================================================================
// Core Types
// ============================================================================

/**
 * Locale configuration for language selector
 */
export interface Locale {
  /** ISO locale code (e.g., 'en', 'es', 'fr') */
  code: string;
  /** Human-readable label (e.g., 'English') */
  label: string;
  /** Emoji flag for visual representation */
  flag: string;
}

/**
 * External link configuration
 */
export interface ExternalLink {
  /** Unique identifier */
  id: string;
  /** Full URL to external resource */
  href: string;
  /** Accessibility label for screen readers */
  label: string;
  /** Icon component to render */
  icon: ReactNode;
}

// ============================================================================
// Component Props Interfaces
// ============================================================================

/**
 * Main Header component props
 */
export interface HeaderProps {
  /** Optional additional CSS class */
  className?: string;
}

/**
 * Language selector dropdown props
 */
export interface LanguageSelectorProps {
  /** Available locales */
  locales: Locale[];
  /** Currently selected locale code */
  currentLocale: string;
  /** Callback when locale is selected */
  onLocaleChange: (localeCode: string) => void;
  /** Optional additional CSS class */
  className?: string;
}

/**
 * User menu (avatar + dropdown) props
 */
export interface UserMenuProps {
  /** User data from auth */
  user: {
    name: string | null;
    email: string;
    image: string | null;
  };
  /** Callback when sign out is clicked */
  onSignOut: () => void;
  /** Optional additional CSS class */
  className?: string;
}

/**
 * Social links component props
 */
export interface SocialLinksProps {
  /** Array of external link configurations */
  links: ExternalLink[];
  /** Optional additional CSS class */
  className?: string;
}

/**
 * Mobile menu drawer props
 */
export interface MobileMenuProps {
  /** Whether menu is open */
  isOpen: boolean;
  /** Callback to close menu */
  onClose: () => void;
  /** Menu content */
  children: ReactNode;
}

/**
 * Auth button props (when not authenticated)
 */
export interface SignInButtonProps {
  /** Callback when button is clicked */
  onClick: () => void;
  /** Loading state */
  isLoading?: boolean;
  /** Optional additional CSS class */
  className?: string;
}

// ============================================================================
// Icon Component Props
// ============================================================================

/**
 * Base props for all icon components
 */
export interface IconProps {
  /** Optional CSS class for styling */
  className?: string;
  /** Icon size (width and height) */
  size?: number;
  /** Accessibility label */
  'aria-label'?: string;
}

// ============================================================================
// State Types
// ============================================================================

/**
 * Header component internal state shape
 */
export interface HeaderState {
  /** Mobile menu open state */
  mobileMenuOpen: boolean;
  /** User dropdown open state */
  userDropdownOpen: boolean;
  /** Language dropdown open state */
  languageDropdownOpen: boolean;
  /** Auth modal open state */
  authModalOpen: boolean;
}

// ============================================================================
// Configuration Constants
// ============================================================================

/**
 * Default locale configuration
 */
export const DEFAULT_LOCALES: Locale[] = [
  { code: 'en', label: 'English', flag: '🇺🇸' },
];

/**
 * External link configuration
 */
export const EXTERNAL_LINKS_CONFIG = {
  github: {
    id: 'github',
    href: 'https://github.com/MuhammadAriyan',
    label: 'GitHub Profile',
  },
  linkedin: {
    id: 'linkedin',
    href: 'https://www.linkedin.com/in/muhammad-aryan',
    label: 'LinkedIn Profile',
  },
} as const;

/**
 * Responsive breakpoints
 */
export const BREAKPOINTS = {
  mobile: 768,
} as const;

/**
 * Header dimensions
 */
export const HEADER_DIMENSIONS = {
  desktop: {
    height: 80,
  },
  mobile: {
    height: 70,
  },
} as const;

/**
 * Maximum display length for user name
 */
export const MAX_USER_NAME_LENGTH = 20;
