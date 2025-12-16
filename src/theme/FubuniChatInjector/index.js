import React from 'react';
import FubuniChat from '../../components/FubuniChat/FubuniChat';

const FubuniChatInjector = () => {
  const backendUrl = typeof window !== 'undefined' && window.location.hostname === 'localhost'
    ? 'http://localhost:8000'
    : 'https://maryanrar-fubuni-chat-api.hf.space';  // Hugging Face Spaces
  
  return <FubuniChat backendUrl={backendUrl} />;
};

export default FubuniChatInjector;