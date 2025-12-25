import React, { useEffect } from 'react';
import SearchButton from './components/SearchButton';

export default function Root({ children }) {
  return (
    <>
      {children}
      <SearchButton />
    </>
  );
}