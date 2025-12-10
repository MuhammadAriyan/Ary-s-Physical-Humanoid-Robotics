import React from 'react';
import FubuniChatInjector from './FubuniChatInjector';

// Default wrapper for the whole Docusaurus site
export default function Root({children}: {children: React.ReactNode}): React.ReactElement {
  return (
    <>
      {children}
      <FubuniChatInjector />
    </>
  );
}