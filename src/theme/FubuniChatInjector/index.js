import React, { useState, useEffect } from 'react';
import { useLocation } from '@docusaurus/router';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

/**
 * Premium floating chat navigation button with smooth animations
 * Links to /chat page instead of opening a chat bubble
 */
const FubuniChatInjector = () => {
  const location = useLocation();
  const [isVisible, setIsVisible] = useState(false);
  const [isHovered, setIsHovered] = useState(false);

  // Don't show on the chat page itself
  const isOnChatPage = location.pathname.includes('/chat');

  useEffect(() => {
    if (isOnChatPage) {
      setIsVisible(false);
      return;
    }

    // Delay appearance for smooth entrance
    const timer = setTimeout(() => {
      setIsVisible(true);
    }, 800);

    return () => clearTimeout(timer);
  }, [isOnChatPage]);

  if (isOnChatPage) {
    return null;
  }

  return (
    <Link
      to="/chat"
      className={`${styles.chatNavButton} ${isVisible ? styles.visible : ''} ${isHovered ? styles.hovered : ''}`}
      onMouseEnter={() => setIsHovered(true)}
      onMouseLeave={() => setIsHovered(false)}
      aria-label="Chat with Fubuni"
    >
      {/* Glow ring effect */}
      <span className={styles.glowRing} aria-hidden="true" />

      {/* Pulse ring */}
      <span className={styles.pulseRing} aria-hidden="true" />

      {/* Chat icon */}
      <span className={styles.iconWrapper} aria-hidden="true">
        💭
      </span>

      {/* Label */}
      <span className={styles.labelWrapper}>
        <span className={styles.chatLabel}>Chat</span>
      </span>

      {/* Shine effect */}
      <span className={styles.shine} aria-hidden="true" />
    </Link>
  );
};

export default FubuniChatInjector;
