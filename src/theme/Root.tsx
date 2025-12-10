import React from 'react';
import FubuniChat from '@site/src/components/FubuniChat/FubuniChat';

// Default wrapper for the whole Docusaurus site
export default function Root({children}: {children: React.ReactNode}): React.ReactElement {
  return (
    <>
      {children}
      <FubuniChat />
    </>
  );
}