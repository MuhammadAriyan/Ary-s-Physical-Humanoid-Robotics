import React from 'react';
import styles from './styles.module.css';

interface ChatMessageProps {
  sender: 'user' | 'fubuni';
  content: string;
}

const ChatMessage: React.FC<ChatMessageProps> = ({ sender, content }) => {
  return (
    <div className={`${styles['fubuni-message']} ${styles[sender]}`}>
      <p className={styles['fubuni-message-content']}>{content}</p>
    </div>
  );
};

export default ChatMessage;