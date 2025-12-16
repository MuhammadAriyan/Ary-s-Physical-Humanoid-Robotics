import React, { useState } from 'react';
import { useAuth } from '../Auth';
import { AuthModal } from '../Auth';
import styles from './AuthButton.module.css';

export const AuthButton: React.FC = () => {
  const { user, isAuthenticated, isLoading } = useAuth();
  const [isModalOpen, setIsModalOpen] = useState(false);

  const openModal = () => setIsModalOpen(true);
  const closeModal = () => setIsModalOpen(false);

  if (isLoading) {
    return <button className={`${styles.authButton} ${styles.loading}`} disabled>
      Loading...
    </button>;
  }

  return (
    <>
      {isAuthenticated && user ? (
        <div className={styles.userContainer}>
          <span className={styles.userName}>{user.name || user.email?.split('@')[0]}</span>
          <div className={styles.avatar}>{user.name?.charAt(0).toUpperCase() || user.email?.charAt(0).toUpperCase()}</div>
        </div>
      ) : (
        <button className={styles.authButton} onClick={openModal}>
          Sign In
        </button>
      )}
      
      <AuthModal 
        isOpen={isModalOpen} 
        onClose={closeModal} 
        onSuccess={() => {}}
      />
    </>
  );
};

export default AuthButton;