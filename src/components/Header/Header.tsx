import React, { useState, useCallback } from 'react';
import Link from '@docusaurus/Link';
import { useAuth, AuthModal } from '../Auth';
import { signOut } from '../../lib/auth-client';
import { UserMenu } from './UserMenu';
import { SocialLinks } from './SocialLinks';
import { MobileMenu } from './MobileMenu';
import { LanguageSelector } from './LanguageSelector';
import { HamburgerIcon } from './icons';
import styles from './Header.module.css';

export interface HeaderProps {
  /** Additional CSS class name */
  className?: string;
}

/**
 * Custom Header component with glassmorphism design
 * Replaces the default Docusaurus navbar
 */
export function Header({ className }: HeaderProps): React.ReactElement {
  const [mobileMenuOpen, setMobileMenuOpen] = useState(false);
  const [authModalOpen, setAuthModalOpen] = useState(false);
  const { user, isAuthenticated, refetch } = useAuth();

  const headerClassName = className
    ? `${styles.header} ${className}`
    : styles.header;

  const handleSignOut = useCallback(async () => {
    try {
      await signOut();
      refetch();
    } catch (error) {
      console.error('Sign out failed:', error);
    }
  }, [refetch]);

  const handleAuthSuccess = useCallback(() => {
    setAuthModalOpen(false);
    refetch();
  }, [refetch]);

  const handleMobileMenuOpen = useCallback(() => {
    setMobileMenuOpen(true);
  }, []);

  const handleMobileMenuClose = useCallback(() => {
    setMobileMenuOpen(false);
  }, []);

  const handleMobileSignIn = useCallback(() => {
    setMobileMenuOpen(false);
    setAuthModalOpen(true);
  }, []);

  const handleMobileSignOut = useCallback(async () => {
    setMobileMenuOpen(false);
    await handleSignOut();
  }, [handleSignOut]);

  return (
    <>
      <header className={headerClassName} role="banner">
        <div className={styles.headerContent}>
          <div className={styles.headerLeft}>
            <Link to="/" className={styles.logo} aria-label="Go to homepage">
              Ary's Physical & Humanoid Robotics
            </Link>
          </div>

          <nav className={styles.headerCenter} role="navigation" aria-label="Main navigation">
            <LanguageSelector />
          </nav>

          <div className={styles.headerRight}>
            <SocialLinks />
            {isAuthenticated && user ? (
              <UserMenu user={user} onSignOut={handleSignOut} />
            ) : (
              <button
                className={styles.signInButton}
                onClick={() => setAuthModalOpen(true)}
                aria-label="Sign in to your account"
              >
                Sign In
              </button>
            )}
          </div>

          {/* Mobile hamburger menu button */}
          <button
            className={styles.mobileMenuButton}
            onClick={handleMobileMenuOpen}
            aria-label="Open navigation menu"
            aria-expanded={mobileMenuOpen}
            type="button"
          >
            <HamburgerIcon size={24} />
          </button>
        </div>
      </header>

      {/* Mobile navigation drawer */}
      <MobileMenu isOpen={mobileMenuOpen} onClose={handleMobileMenuClose}>
        {/* Language Section */}
        <div className={styles.mobileNavSection}>
          <span className={styles.mobileNavTitle}>Language</span>
          <LanguageSelector />
        </div>

        {/* Social Links Section */}
        <div className={styles.mobileNavSection}>
          <span className={styles.mobileNavTitle}>Connect</span>
          <SocialLinks />
        </div>

        {/* Auth Section */}
        <div className={styles.mobileNavSection}>
          <span className={styles.mobileNavTitle}>Account</span>
          {isAuthenticated && user ? (
            <UserMenu user={user} onSignOut={handleMobileSignOut} />
          ) : (
            <button
              className={styles.mobileSignInButton}
              onClick={handleMobileSignIn}
              aria-label="Sign in to your account"
              type="button"
            >
              Sign In
            </button>
          )}
        </div>
      </MobileMenu>

      <AuthModal
        isOpen={authModalOpen}
        onClose={() => setAuthModalOpen(false)}
        onSuccess={handleAuthSuccess}
      />
    </>
  );
}

export default Header;
