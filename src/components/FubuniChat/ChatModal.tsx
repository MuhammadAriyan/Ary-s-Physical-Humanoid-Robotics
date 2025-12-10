import React from 'react';
import styles from './styles.module.css';

interface ChatModalProps {
  isOpen: boolean;
  onClose: () => void;
  onMinimize: () => void;
  children: React.ReactNode;
}

const ChatModal: React.FC<ChatModalProps> = ({ isOpen, onClose, onMinimize, children }) => {
  if (!isOpen) return null;

  return (
    <div className={styles['fubuni-chat-modal']}>
      <div className={styles['fubuni-chat-modal-content']}>
        <div className={styles['fubuni-chat-modal-header']}>
          <h3 className={styles['fubuni-chat-title']}>Fubuni Chat</h3>
          <div className={styles['fubuni-chat-controls']}>
            <button
              className={styles['fubuni-chat-btn']}
              onClick={onMinimize}
            >
              Minimize
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
    </div>
  );
};

export default ChatModal;