import React from 'react';
import styles from './styles.module.css';

interface ChatDrawerProps {
  isOpen: boolean;
  onClose: () => void;
  onExpand: () => void;
  children: React.ReactNode;
}

const ChatDrawer: React.FC<ChatDrawerProps> = ({ isOpen, onClose, onExpand, children }) => {
  if (!isOpen) return null;

  return (
    <div className={`${styles['fubuni-chat-drawer']}`}>
      <div className={styles['fubuni-chat-header']}>
        <h3 className={styles['fubuni-chat-title']}>Fubuni Chat</h3>
        <div className={styles['fubuni-chat-controls']}>
          <button
            className={styles['fubuni-chat-btn']}
            onClick={onExpand}
          >
            Expand
          </button>
          <button
            className={styles['fubuni-chat-btn']}
            onClick={onClose}
          >
            Close
          </button>
        </div>
      </div>

      <div className={styles['fubuni-chat-messages']}>
        {children}
      </div>
    </div>
  );
};

export default ChatDrawer;