import React from 'react';
import FubuniChat from '../../components/FubuniChat/FubuniChat';

const FubuniChatInjector = () => {
  const backendUrl = typeof window !== 'undefined' && window.location.hostname === 'localhost'
    ? 'http://localhost:8000'
    : 'https://humanoid-robotics-book-production.up.railway.app';  // Updated Railway URL
  
  return <FubuniChat backendUrl={backendUrl} />;
};

export default FubuniChatInjector;