/**
 * Floating TranslateButton wrapper for global use.
 * Appears on doc pages and uses the current URL path as chapter slug.
 */

import { useState, useEffect, useCallback } from 'react';
import { useLocation } from '@docusaurus/router';
import { useTranslate } from '../../hooks/useTranslate';
import styles from './FloatingTranslateButton.module.css';

// Icons
const TranslateIcon = () => (
  <svg viewBox="0 0 24 24" fill="currentColor">
    <path d="M12.87 15.07l-2.54-2.51.03-.03c1.74-1.94 2.98-4.17 3.71-6.53H17V4h-7V2H8v2H1v1.99h11.17C11.5 7.92 10.44 9.75 9 11.35 8.07 10.32 7.3 9.19 6.69 8h-2c.73 1.63 1.73 3.17 2.98 4.56l-5.09 5.02L4 19l5-5 3.11 3.11.76-2.04zM18.5 10h-2L12 22h2l1.12-3h4.75L21 22h2l-4.5-12zm-2.62 7l1.62-4.33L19.12 17h-3.24z" />
  </svg>
);

const CloseIcon = () => (
  <svg viewBox="0 0 24 24" fill="currentColor">
    <path d="M19 6.41L17.59 5 12 10.59 6.41 5 5 6.41 10.59 12 5 17.59 6.41 19 12 13.41 17.59 19 19 17.59 13.41 12z" />
  </svg>
);

export default function FloatingTranslateButton() {
  const location = useLocation();
  const [isOpen, setIsOpen] = useState(false);
  const [isDocPage, setIsDocPage] = useState(false);
  const [chapterSlug, setChapterSlug] = useState('');

  const {
    translatedContent,
    isLoading,
    isPending,
    error,
    estimatedSeconds,
    requestTranslation,
    clearTranslation,
  } = useTranslate();

  // Determine if we're on a doc page and extract chapter slug
  useEffect(() => {
    const path = location.pathname;
    const isDoc = path.startsWith('/docs/');
    setIsDocPage(isDoc);

    if (isDoc) {
      // Extract slug from path: /docs/chapter-1/section -> chapter-1-section
      const slug = path
        .replace(/^\/docs\//, '')
        .replace(/\/$/, '')
        .replace(/\//g, '-') || 'index';
      setChapterSlug(slug);
    }
  }, [location.pathname]);

  const handleTranslate = useCallback(async () => {
    if (!chapterSlug) return;

    // Get the main content from the page
    const contentElement = document.querySelector('article') ||
      document.querySelector('.markdown') ||
      document.querySelector('main');

    if (!contentElement) {
      console.error('Could not find page content to translate');
      return;
    }

    // Clone and clean content
    const clonedContent = contentElement.cloneNode(true) as HTMLElement;
    const elementsToRemove = clonedContent.querySelectorAll(
      'pre, code, .hash-link, nav, .pagination-nav, .theme-code-block, .translateContainer'
    );
    elementsToRemove.forEach((el) => el.remove());

    const textContent = clonedContent.textContent?.trim() || '';

    if (textContent.length < 50) {
      console.error('Not enough content to translate');
      return;
    }

    await requestTranslation(chapterSlug, textContent);
  }, [chapterSlug, requestTranslation]);

  const togglePanel = () => setIsOpen((prev) => !prev);

  // Don't render on non-doc pages
  if (!isDocPage) return null;

  return (
    <div className={styles.floatingContainer}>
      {isOpen && (
        <div className={styles.panel}>
          <div className={styles.header}>
            <h3 className={styles.title}>Translate to Urdu</h3>
            <button
              className={styles.closeBtn}
              onClick={togglePanel}
              aria-label="Close"
            >
              <CloseIcon />
            </button>
          </div>

          <div className={styles.content}>
            {!translatedContent && !isPending && (
              <button
                className={styles.translateBtn}
                onClick={handleTranslate}
                disabled={isLoading || isPending}
              >
                {isLoading ? 'Loading...' : 'Translate This Page'}
              </button>
            )}

            {isPending && (
              <div className={styles.pending}>
                <div className={styles.progressBar}>
                  <div className={styles.progressFill} />
                </div>
                <p>
                  {estimatedSeconds
                    ? `Translating... (est. ${estimatedSeconds}s)`
                    : 'Translating...'}
                </p>
              </div>
            )}

            {error && <div className={styles.error}>{error}</div>}

            {translatedContent && (
              <div className={styles.translatedSection}>
                <div className={styles.urduText}>{translatedContent}</div>
                <button
                  className={styles.clearBtn}
                  onClick={clearTranslation}
                >
                  Clear Translation
                </button>
              </div>
            )}
          </div>
        </div>
      )}

      <button
        className={styles.fab}
        onClick={togglePanel}
        aria-label={isOpen ? 'Close translation' : 'Translate page'}
        title="Translate to Urdu"
      >
        {isOpen ? <CloseIcon /> : <TranslateIcon />}
      </button>
    </div>
  );
}
