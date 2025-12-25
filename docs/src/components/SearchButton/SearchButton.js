import React, { useState, useEffect } from 'react';
import { useSearchData } from '@docusaurus/theme-common';
import { DocSearchButton } from './DocSearchButton';

export default function SearchButton() {
  return (
    <div className="search-button-container">
      <DocSearchButton />
    </div>
  );
}