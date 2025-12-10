import React from 'react';
import styles from './styles.module.css';

interface FubuniBubbleProps {
  onClick: () => void;
}

const FubuniBubble: React.FC<FubuniBubbleProps> = ({ onClick }) => {
  return (
    <div
      className={`${styles['fubuni-bubble']} ${styles.pulse}`}
      onClick={onClick}
      title="Chat with Fubuni"
    >
      💬
    </div>
  );
};

export default FubuniBubble;