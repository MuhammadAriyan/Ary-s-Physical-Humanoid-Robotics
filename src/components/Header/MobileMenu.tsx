import React, { useEffect, useCallback, useRef } from 'react';
import { CloseIcon } from './icons';
import styles from './MobileMenu.module.css';

export interface MobileMenuProps {
  /** Whether the menu is open */
  isOpen: boolean;
  /** Callback to close the menu */
  onClose: () => void;
  /** Content to render inside the menu */
  children: React.ReactNode;
}

/**
 * MobileMenu component - slide-in drawer navigation for mobile devices
 * Features backdrop overlay, glassmorphism styling, and smooth animations
 */
export function MobileMenu({
  isOpen,
  onClose,
  children,
}: MobileMenuProps): React.ReactElement | null {
  const menuRef = useRef<HTMLDivElement>(null);
  const closeButtonRef = useRef<HTMLButtonElement>(null);

  // Handle escape key to close menu
  const handleKeyDown = useCallback(
    (event: KeyboardEvent) => {
      if (event.key === 'Escape') {
        onClose();
      }
    },
    [onClose]
  );

  // Focus trap and keyboard handling
  useEffect(() => {
    if (isOpen) {
      document.addEventListener('keydown', handleKeyDown);
      // Prevent body scroll when menu is open
      document.body.style.overflow = 'hidden';
      // Focus close button when menu opens
      closeButtonRef.current?.focus();
    } else {
      document.body.style.overflow = '';
    }

    return () => {
      document.removeEventListener('keydown', handleKeyDown);
      document.body.style.overflow = '';
    };
  }, [isOpen, handleKeyDown]);

  // Handle backdrop click
  const handleBackdropClick = useCallback(
    (event: React.MouseEvent<HTMLDivElement>) => {
      if (event.target === event.currentTarget) {
        onClose();
      }
    },
    [onClose]
  );

  if (!isOpen) {
    return null;
  }

  return (
    <div
      className={styles.backdrop}
      onClick={handleBackdropClick}
      role="presentation"
      aria-hidden="true"
    >
      <div
        ref={menuRef}
        className={styles.drawer}
        role="dialog"
        aria-modal="true"
        aria-label="Mobile navigation menu"
      >
        <div className={styles.drawerHeader}>
          <button
            ref={closeButtonRef}
            className={styles.closeButton}
            onClick={onClose}
            aria-label="Close navigation menu"
            type="button"
          >
            <CloseIcon size={24} />
          </button>
        </div>
        <nav className={styles.drawerContent} role="navigation" aria-label="Mobile navigation">
          {children}
        </nav>
      </div>
    </div>
  );
}

export default MobileMenu;
