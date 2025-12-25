import React from 'react';
import SearchButton from '@site/src/components/SearchButton';

export default function LayoutWrapper(props) {
  return (
    <>
      {props.children}
      <SearchButton />
    </>
  );
}