import React from 'react';
import FubuniChat from '../../components/FubuniChat/FubuniChat';

const FubuniChatInjector = () => {
  const backendUrl = process.env.NODE_ENV === 'production' 
    ? 'https://your-app-name.railway.app'  // Replace with your Railway URL
    : 'http://localhost:8000';
  
  return <FubuniChat backendUrl={backendUrl} />;
};

export default FubuniChatInjector;