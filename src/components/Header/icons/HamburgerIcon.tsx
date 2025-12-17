import React from 'react';

export interface HamburgerIconProps {
  /** Additional CSS class name */
  className?: string;
  /** Icon size in pixels */
  size?: number;
  /** ARIA label for accessibility */
  'aria-label'?: string;
}

/**
 * Hamburger menu SVG icon component
 * Three horizontal lines representing a menu toggle
 */
export const HamburgerIcon: React.FC<HamburgerIconProps> = ({
  className,
  size = 24,
  'aria-label': ariaLabel,
}) => (
  <svg
    className={className}
    width={size}
    height={size}
    viewBox="0 0 24 24"
    fill="none"
    stroke="currentColor"
    strokeWidth="2"
    strokeLinecap="round"
    strokeLinejoin="round"
    aria-label={ariaLabel}
    role={ariaLabel ? 'img' : 'presentation'}
  >
    <line x1="3" y1="6" x2="21" y2="6" />
    <line x1="3" y1="12" x2="21" y2="12" />
    <line x1="3" y1="18" x2="21" y2="18" />
  </svg>
);

export default HamburgerIcon;
