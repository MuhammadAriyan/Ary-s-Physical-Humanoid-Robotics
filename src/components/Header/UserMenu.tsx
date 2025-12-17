import React, { useState, useRef, useEffect, useCallback } from 'react';
import type { AuthUser } from '../../lib/auth-client';
import styles from './UserMenu.module.css';

export interface UserMenuProps {
  /** The authenticated user object */
  user: AuthUser;
  /** Callback when sign out is clicked */
  onSignOut: () => void;
  /** Additional CSS class name */
  className?: string;
}

/**
 * Truncates a string to the specified length with ellipsis
 */
function truncateString(str: string, maxLength: number): string {
  if (str.length <= maxLength) return str;
  return `${str.slice(0, maxLength)}...`;
}

/**
 * Gets the user's initial for the avatar
 */
function getUserInitial(user: AuthUser): string {
  if (user.name && user.name.length > 0) {
    return user.name.charAt(0).toUpperCase();
  }
  if (user.email && user.email.length > 0) {
    return user.email.charAt(0).toUpperCase();
  }
  return '?';
}

/**
 * Gets the display name for the user (truncated to 20 chars)
 */
function getDisplayName(user: AuthUser): string {
  const name = user.name || user.email || 'User';
  return truncateString(name, 20);
}

/**
 * UserMenu component displays the authenticated user's avatar
 * with a dropdown menu for user actions
 */
export function UserMenu({ user, onSignOut, className }: UserMenuProps): React.ReactElement {
  const [isOpen, setIsOpen] = useState(false);
  const menuRef = useRef<HTMLDivElement>(null);
  const buttonRef = useRef<HTMLButtonElement>(null);

  const initial = getUserInitial(user);
  const displayName = getDisplayName(user);

  // Handle click outside to close dropdown
  const handleClickOutside = useCallback((event: MouseEvent) => {
    if (
      menuRef.current &&
      !menuRef.current.contains(event.target as Node) &&
      buttonRef.current &&
      !buttonRef.current.contains(event.target as Node)
    ) {
      setIsOpen(false);
    }
  }, []);

  // Handle escape key to close dropdown
  const handleKeyDown = useCallback((event: KeyboardEvent) => {
    if (event.key === 'Escape') {
      setIsOpen(false);
      buttonRef.current?.focus();
    }
  }, []);

  useEffect(() => {
    if (isOpen) {
      document.addEventListener('mousedown', handleClickOutside);
      document.addEventListener('keydown', handleKeyDown);
    }

    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
      document.removeEventListener('keydown', handleKeyDown);
    };
  }, [isOpen, handleClickOutside, handleKeyDown]);

  const toggleMenu = () => {
    setIsOpen((prev) => !prev);
  };

  const handleSignOut = () => {
    setIsOpen(false);
    onSignOut();
  };

  const containerClassName = className
    ? `${styles.userMenu} ${className}`
    : styles.userMenu;

  return (
    <div className={containerClassName}>
      <button
        ref={buttonRef}
        className={styles.avatarButton}
        onClick={toggleMenu}
        aria-expanded={isOpen}
        aria-haspopup="menu"
        aria-label={`User menu for ${displayName}`}
        title={displayName}
      >
        {user.image ? (
          <img
            src={user.image}
            alt={`${displayName}'s avatar`}
            className={styles.avatarImage}
          />
        ) : (
          <span className={styles.avatarInitial}>{initial}</span>
        )}
      </button>

      {isOpen && (
        <div
          ref={menuRef}
          className={styles.dropdown}
          role="menu"
          aria-label="User actions"
        >
          <div className={styles.dropdownHeader}>
            <span className={styles.userName} title={user.name || user.email || ''}>
              {displayName}
            </span>
            {user.email && (
              <span className={styles.userEmail} title={user.email}>
                {truncateString(user.email, 25)}
              </span>
            )}
          </div>
          <div className={styles.dropdownDivider} role="separator" />
          <button
            className={styles.dropdownItem}
            onClick={handleSignOut}
            role="menuitem"
          >
            Sign Out
          </button>
        </div>
      )}
    </div>
  );
}

export default UserMenu;
