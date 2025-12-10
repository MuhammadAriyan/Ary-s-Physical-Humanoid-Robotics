import React from 'react';
import FubuniChat from '@site/src/components/FubuniChat/FubuniChat';

const FubuniChatInjector = () => {
  const backendUrl = process.env.NODE_ENV === 'production' 
    ? process.env.REACT_APP_BACKEND_URL || 'https://your-backend-url.railway.app'
    : 'http://localhost:8000';
  
  return <FubuniChat backendUrl={backendUrl} />;
};

export default FubuniChatInjector;