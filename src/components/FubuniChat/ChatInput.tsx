import React from 'react';
import styles from './styles.module.css';

interface ChatInputProps {
  value: string;
  onChange: (value: string) => void;
  onSubmit: (e: React.FormEvent) => void;
  isLoading: boolean;
}

const ChatInput: React.FC<ChatInputProps> = ({ value, onChange, onSubmit, isLoading }) => {
  return (
    <form onSubmit={onSubmit} className={styles['fubuni-chat-input-container']}>
      <input
        type="text"
        value={value}
        onChange={(e) => onChange(e.target.value)}
        className={styles['fubuni-chat-input']}
        placeholder="Type your message..."
        disabled={isLoading}
      />
      <button
        type="submit"
        className={styles['fubuni-chat-send-btn']}
        disabled={isLoading || !value.trim()}
      >
        {isLoading ? 'Sending...' : 'Send'}
      </button>
    </form>
  );
};

export default ChatInput;