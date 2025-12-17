import React from 'react';

export interface CloseIconProps {
  /** Additional CSS class name */
  className?: string;
  /** Icon size in pixels */
  size?: number;
  /** ARIA label for accessibility */
  'aria-label'?: string;
}

/**
 * Close (X) SVG icon component
 * Two diagonal lines forming an X shape
 */
export const CloseIcon: React.FC<CloseIconProps> = ({
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
    <line x1="18" y1="6" x2="6" y2="18" />
    <line x1="6" y1="6" x2="18" y2="18" />
  </svg>
);

export default CloseIcon;
