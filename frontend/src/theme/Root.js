import React from 'react';
import ChatWidget from '../components/ChatWidget';

// Default theme wrapper
const Root = ({children}) => {
  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
};

export default Root;