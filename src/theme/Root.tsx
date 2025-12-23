import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import FubuniChatInjector from './FubuniChatInjector';
import { AuthProvider } from '../components/Auth/AuthProvider';

// Default wrapper for the whole Docusaurus site
export default function Root({children}: {children: React.ReactNode}): React.ReactElement {
  return (
    <BrowserOnly fallback={<>{children}</>}>
      {() => (
        <AuthProvider>
          {children}
          <FubuniChatInjector />
        </AuthProvider>
      )}
    </BrowserOnly>
  );
}