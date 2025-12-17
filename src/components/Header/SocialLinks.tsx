import React from 'react';
import { GitHubIcon, LinkedInIcon } from './icons';
import styles from './SocialLinks.module.css';

export interface SocialLink {
  /** Unique identifier for the link */
  id: string;
  /** URL to navigate to */
  href: string;
  /** Tooltip/title text */
  title: string;
  /** ARIA label for accessibility */
  ariaLabel: string;
  /** Icon component to render */
  icon: React.ReactElement;
}

export interface SocialLinksProps {
  /** Additional CSS class name */
  className?: string;
}

/**
 * Social links configuration
 */
const socialLinks: SocialLink[] = [
  {
    id: 'github',
    href: 'https://github.com/MuhammadAriyan',
    title: 'GitHub Profile',
    ariaLabel: 'Visit GitHub profile (opens in new tab)',
    icon: <GitHubIcon size={20} />,
  },
  {
    id: 'linkedin',
    href: 'https://linkedin.com/in/muhammad-aryan',
    title: 'LinkedIn Profile',
    ariaLabel: 'Visit LinkedIn profile (opens in new tab)',
    icon: <LinkedInIcon size={20} />,
  },
];

/**
 * SocialLinks component displays social media icon links
 */
export function SocialLinks({ className }: SocialLinksProps): React.ReactElement {
  const containerClassName = className
    ? `${styles.socialLinks} ${className}`
    : styles.socialLinks;

  return (
    <div className={containerClassName} role="navigation" aria-label="Social links">
      {socialLinks.map((link) => (
        <a
          key={link.id}
          href={link.href}
          className={styles.socialLink}
          target="_blank"
          rel="noopener noreferrer"
          title={link.title}
          aria-label={link.ariaLabel}
        >
          {link.icon}
        </a>
      ))}
    </div>
  );
}

export default SocialLinks;
