import React from 'react';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

/**
 * Floating chat navigation button - replaces the old Fubuni bubble
 * Links to the /chat page instead of opening a modal
 */
const FubuniChatInjector = () => {
  return (
    <Link to="/chat" className={styles.chatNavButton} title="Chat with Fubuni">
      <span className={styles.chatIcon}>💬</span>
      <span className={styles.chatLabel}>Chat</span>
    </Link>
  );
};

export default FubuniChatInjector;