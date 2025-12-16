import React from 'react';
import styles from './styles.module.css';

interface ChatModalProps {
  isOpen: boolean;
  onClose: () => void;
  onMinimize: () => void;
  onSignOut?: () => void;
  userName?: string | null;
  children: React.ReactNode;
}

const ChatModal: React.FC<ChatModalProps> = ({ isOpen, onClose, onMinimize, onSignOut, userName, children }) => {
  if (!isOpen) return null;

  return (
    <div className={styles['fubuni-chat-modal']}>
      <div className={styles['fubuni-chat-modal-content']}>
        <div className={styles['fubuni-chat-modal-header']}>
          <h3 className={styles['fubuni-chat-title']}>Fubuni Chat</h3>
          <div className={styles['fubuni-chat-controls']}>
            {userName && onSignOut && (
              <button
                className={styles['fubuni-chat-btn']}
                onClick={onSignOut}
                aria-label="Sign out"
                title={`Sign out (${userName})`}
              >
                <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <path d="M9 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h4"/>
                  <polyline points="16 17 21 12 16 7"/>
                  <line x1="21" y1="12" x2="9" y2="12"/>
                </svg>
              </button>
            )}
            <button
              className={styles['fubuni-chat-btn']}
              onClick={onMinimize}
              aria-label="Minimize"
            >
              <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <path d="M8 3v3a2 2 0 0 1-2 2H3m18 0h-3a2 2 0 0 1-2-2V3m0 18v-3a2 2 0 0 1 2-2h3M3 16h3a2 2 0 0 1 2 2v3"/>
              </svg>
            </button>
            <button
              className={styles['fubuni-chat-btn']}
              onClick={onClose}
              aria-label="Close"
            >
              <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <path d="M18 6L6 18M6 6l12 12"/>
              </svg>
            </button>
          </div>
        </div>

        <div className={styles['fubuni-chat-messages']}>
          {children}
        </div>
      </div>
    </div>
  );
};

export default ChatModal;