import React from 'react';
import styles from './styles.module.css';

interface ChatHistoryProps {
  sessionId: string;
}

const ChatHistory: React.FC<ChatHistoryProps> = ({ sessionId }) => {
  // This would typically fetch chat history from the backend API
  // For now, we'll just return a placeholder
  return (
    <div className={styles['chat-history']}>
      <h4>Chat History for Session: {sessionId}</h4>
      <p>Chat history would be displayed here.</p>
    </div>
  );
};

export default ChatHistory;