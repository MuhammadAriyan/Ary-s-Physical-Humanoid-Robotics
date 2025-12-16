import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import { useLocation } from '@docusaurus/router';
import { AuthButton } from '../../components/Auth';
import styles from './Navbar.module.css';

// Placeholder for the actual Docusaurus Navbar component
const OriginalNavbar = React.lazy(() => import('@theme-original/Navbar'));

export default function NavbarWrapper(props) {
  const location = useLocation();
  const [showAuthButton, setShowAuthButton] = useState(false);

  // Show auth button on specific pages (like the homepage or docs)
  useEffect(() => {
    const shouldShowAuth = location.pathname === '/' ||
                          location.pathname.startsWith('/docs') ||
                          location.pathname.startsWith('/blog');
    setShowAuthButton(shouldShowAuth);
  }, [location]);

  return (
    <>
      <OriginalNavbar {...props} />
      {showAuthButton && (
        <div className={styles.authButtonOverlay}>
          <AuthButton />
        </div>
      )}
    </>
  );
}