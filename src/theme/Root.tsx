import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import FubuniChatInjector from './FubuniChatInjector';

// Default wrapper for the whole Docusaurus site
// AuthProvider removed temporarily to fix hydration issues
export default function Root({children}: {children: React.ReactNode}): React.ReactElement {
  return (
    <>
      {children}
      <BrowserOnly fallback={null}>
        {() => <FubuniChatInjector />}
      </BrowserOnly>
    </>
  );
}