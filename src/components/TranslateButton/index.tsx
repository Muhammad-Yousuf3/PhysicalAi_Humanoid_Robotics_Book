/**
 * TranslateButton component for translating chapter content to Urdu.
 * Can be used in MDX files at the start of chapters.
 */

import { useEffect, useCallback } from 'react';
import { useTranslate } from '../../hooks/useTranslate';
import styles from './styles.module.css';

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

interface TranslateButtonProps {
  /** Chapter slug used for API requests */
  chapterSlug: string;
  /** Optional custom label */
  label?: string;
}

export default function TranslateButton({
  chapterSlug,
  label = 'Translate to Urdu',
}: TranslateButtonProps) {
  const {
    translatedContent,
    isLoading,
    isPending,
    error,
    estimatedSeconds,
    requestTranslation,
    checkTranslation,
    clearTranslation,
  } = useTranslate();

  // Check if translation exists on mount
  useEffect(() => {
    checkTranslation(chapterSlug);
  }, [chapterSlug, checkTranslation]);

  const handleTranslate = useCallback(async () => {
    // Get the main content from the page
    const contentElement = document.querySelector('article') ||
      document.querySelector('.markdown') ||
      document.querySelector('main');

    if (!contentElement) {
      console.error('Could not find page content to translate');
      return;
    }

    // Get text content, excluding code blocks and navigation
    const clonedContent = contentElement.cloneNode(true) as HTMLElement;

    // Remove elements we don't want to translate
    const elementsToRemove = clonedContent.querySelectorAll(
      'pre, code, .hash-link, nav, .pagination-nav, .theme-code-block'
    );
    elementsToRemove.forEach((el) => el.remove());

    const textContent = clonedContent.textContent?.trim() || '';

    if (textContent.length < 50) {
      console.error('Not enough content to translate');
      return;
    }

    await requestTranslation(chapterSlug, textContent);
  }, [chapterSlug, requestTranslation]);

  const getStatusText = () => {
    if (isLoading) return 'Loading...';
    if (isPending) {
      return estimatedSeconds
        ? `Translating... (est. ${estimatedSeconds}s remaining)`
        : 'Translating...';
    }
    if (translatedContent) return 'Translation available';
    return '';
  };

  return (
    <div className={styles.translateContainer}>
      <div className={styles.translateHeader}>
        <button
          className={styles.translateButton}
          onClick={handleTranslate}
          disabled={isLoading || isPending || !!translatedContent}
        >
          <TranslateIcon />
          {translatedContent ? 'Translated' : label}
        </button>
        <span className={styles.statusText}>{getStatusText()}</span>
      </div>

      {isPending && (
        <div className={styles.progressBar}>
          <div className={styles.progressFill} />
        </div>
      )}

      {error && <div className={styles.errorMessage}>{error}</div>}

      {translatedContent && (
        <div className={styles.translatedContent}>
          <div className={styles.translatedHeader}>
            <h4 className={styles.translatedTitle}>Urdu Translation</h4>
            <button
              className={styles.closeButton}
              onClick={clearTranslation}
              title="Hide translation"
              aria-label="Hide translation"
            >
              <CloseIcon />
            </button>
          </div>
          <div className={styles.urduText}>{translatedContent}</div>
        </div>
      )}
    </div>
  );
}
