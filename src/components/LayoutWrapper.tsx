import React from 'react';
import RAGChatbot from './RAGChatbot';

type LayoutWrapperProps = {
  children: React.ReactNode;
};

const LayoutWrapper: React.FC<LayoutWrapperProps> = ({ children }) => {
  return (
    <>
      {children}
      <RAGChatbot />
    </>
  );
};

export default LayoutWrapper;