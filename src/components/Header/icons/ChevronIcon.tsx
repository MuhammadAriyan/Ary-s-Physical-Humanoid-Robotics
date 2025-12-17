import React from 'react';

export interface ChevronIconProps {
  /** Additional CSS class name */
  className?: string;
  /** Icon size in pixels */
  size?: number;
  /** Direction the chevron points */
  direction?: 'up' | 'down' | 'left' | 'right';
  /** ARIA label for accessibility */
  'aria-label'?: string;
}

/**
 * Chevron SVG icon component
 * Arrow pointing in specified direction (default: down)
 */
export const ChevronIcon: React.FC<ChevronIconProps> = ({
  className,
  size = 16,
  direction = 'down',
  'aria-label': ariaLabel,
}) => {
  const rotationMap = {
    down: 0,
    up: 180,
    left: 90,
    right: -90,
  };

  return (
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
      style={{ transform: `rotate(${rotationMap[direction]}deg)` }}
      aria-label={ariaLabel}
      role={ariaLabel ? 'img' : 'presentation'}
    >
      <polyline points="6 9 12 15 18 9" />
    </svg>
  );
};

export default ChevronIcon;
