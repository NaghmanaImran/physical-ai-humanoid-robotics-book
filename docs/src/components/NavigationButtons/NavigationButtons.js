import React from 'react';
import { useDocsData, useActiveDocContext } from '@docusaurus/plugin-content-docs/client';
import Link from '@docusaurus/Link';
import clsx from 'clsx';
import styles from './NavigationButtons.module.css';

// This component provides next and previous chapter navigation
const NavigationButtons = () => {
  const { activeDoc } = useActiveDocContext();
  if (!activeDoc) {
    return null;
  }

  const docsData = useDocsData(activeDoc.sidebarName);
  if (!docsData) {
    return null;
  }

  // Find the current document in the sidebar
  const findCurrentDoc = (items, currentId) => {
    if (!items || !Array.isArray(items)) {
      return null;
    }

    for (const item of items) {
      if (item.type === 'doc') {
        if (item.id === currentId) {
          return { item, parent: null };
        }
      } else if (item.type === 'category' && item.items && Array.isArray(item.items)) {
        const found = findCurrentDoc(item.items, currentId);
        if (found) {
          return { item: found.item, parent: item };
        }
      } else if (item.type === 'link') {
        // Links don't have IDs, so we can't match them directly
      }
    }
    return null;
  };

  // Flatten sidebar to get the linear order of documents
  const flattenSidebar = (items) => {
    if (!items || !Array.isArray(items)) {
      return [];
    }

    let flatItems = [];

    for (const item of items) {
      if (item.type === 'doc') {
        flatItems.push(item);
      } else if (item.type === 'category' && item.items && Array.isArray(item.items)) {
        flatItems = flatItems.concat(flattenSidebar(item.items));
      }
    }

    return flatItems;
  };

  // Get current document info
  const currentDoc = findCurrentDoc(docsData.sidebar, activeDoc.id);
  if (!currentDoc) {
    return null;
  }

  // Get flattened list of documents for navigation
  const allDocs = flattenSidebar(docsData.sidebar);
  const currentIndex = allDocs.findIndex(doc => doc.id === activeDoc.id);

  if (currentIndex === -1) {
    return null;
  }

  // Get previous and next documents
  const prevDoc = currentIndex > 0 ? allDocs[currentIndex - 1] : null;
  const nextDoc = currentIndex < allDocs.length - 1 ? allDocs[currentIndex + 1] : null;

  return (
    <div className={styles.navButtons}>
      <div className={styles.prevButton}>
        {prevDoc && (
          <Link to={prevDoc.permalink}>
            &larr; Previous: {prevDoc.title}
          </Link>
        )}
      </div>
      <div className={styles.nextButton}>
        {nextDoc && (
          <Link to={nextDoc.permalink}>
            Next: {nextDoc.title} &rarr;
          </Link>
        )}
      </div>
    </div>
  );
};

export default NavigationButtons;