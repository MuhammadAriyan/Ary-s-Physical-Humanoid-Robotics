import React from 'react';
import FubuniChatInjector from './FubuniChatInjector';
import LanguageToggle from '../components/LanguageToggle';
import { AuthProvider } from '../components/Auth';

// Default wrapper for the whole Docusaurus site
export default function Root({children}: {children: React.ReactNode}): React.ReactElement {
  return (
    <AuthProvider>
      {children}
      <LanguageToggle />
      <FubuniChatInjector />
    </AuthProvider>
  );
}