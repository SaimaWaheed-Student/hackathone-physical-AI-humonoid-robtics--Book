import React from 'react';
import ChatbotWidget from '@site/src/components/ChatbotWidget';

// Default implementation, that you can customize
// This is the "swizzled" Root component
function Root({ children }) {
  return (
    <>
      {children}
      <ChatbotWidget />
    </>
  );
}

export default Root;