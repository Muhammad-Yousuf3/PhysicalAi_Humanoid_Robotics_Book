/**
 * usePageTranslation - Hook for translating entire page content.
 *
 * Provides DOM-level translation by:
 * - Collecting ALL text nodes from the page
 * - Batching translation API calls
 * - Updating DOM with translated content
 * - Preserving original text for restoration
 * - Avoiding double-translation and infinite loops
 */

import { useEffect, useRef, useCallback } from 'react';
import { usePageTranslationContext } from '../contexts/PageTranslationContext';
import { apiClient } from '../utils/api';
import type { TranslationResult, TranslationRequest } from '../types/api';
import { isTranslationComplete } from '../types/api';

const TRANSLATED_ATTR = 'data-translated';
const ORIGINAL_TEXT_ATTR = 'data-original-text';
const SEPARATOR = '|||SEP|||';

// Debug mode - set to true to see what's being collected
const DEBUG = true;

/**
 * Tags to completely exclude from translation (only essential ones).
 */
const EXCLUDE_TAGS = new Set([
  'SCRIPT',
  'STYLE',
  'NOSCRIPT',
]);

/**
 * Check if element should be excluded (minimal check).
 */
function shouldExcludeElement(element: Element): boolean {
  // Walk up the tree checking each ancestor
  let current: Element | null = element;

  while (current) {
    const tagName = current.tagName;

    // Exclude script, style, noscript
    if (EXCLUDE_TAGS.has(tagName)) {
      return true;
    }

    // Exclude code blocks (but not inline code in paragraphs for now)
    if (tagName === 'PRE') {
      return true;
    }

    // Exclude SVG elements
    if (tagName === 'SVG' || current.namespaceURI === 'http://www.w3.org/2000/svg') {
      return true;
    }

    // Exclude explicit no-translate markers
    if (current.hasAttribute && current.hasAttribute('data-no-translate')) {
      return true;
    }

    current = current.parentElement;
  }

  return false;
}

/**
 * Check if a text node should be translated.
 */
function shouldTranslateTextNode(node: Text): boolean {
  const text = node.textContent?.trim();

  // Skip empty or whitespace-only nodes
  if (!text || text.length === 0) {
    return false;
  }

  // Must have a parent element
  if (!node.parentElement) {
    return false;
  }

  // Check if parent should be excluded
  if (shouldExcludeElement(node.parentElement)) {
    return false;
  }

  // Skip if already translated
  if (node.parentElement.hasAttribute(TRANSLATED_ATTR)) {
    return false;
  }

  return true;
}

/**
 * Collect ALL translatable text nodes from the ENTIRE page.
 * Simple approach: scan entire body, only exclude script/style/pre/svg.
 */
function collectTextNodes(rootElement: Element): Text[] {
  const textNodes: Text[] = [];

  if (DEBUG) {
    console.log('[PageTranslation] Starting text node collection...');
    console.log('[PageTranslation] Scanning entire:', rootElement.tagName);
  }

  // Use TreeWalker to traverse ALL text nodes in the body
  const walker = document.createTreeWalker(
    rootElement,
    NodeFilter.SHOW_TEXT,
    {
      acceptNode: (node) => {
        const textNode = node as Text;
        const shouldAccept = shouldTranslateTextNode(textNode);

        if (DEBUG && shouldAccept && textNode.textContent) {
          const preview = textNode.textContent.trim().substring(0, 50);
          console.log('[PageTranslation] ✓:', preview);
        }

        return shouldAccept
          ? NodeFilter.FILTER_ACCEPT
          : NodeFilter.FILTER_SKIP;
      },
    }
  );

  let currentNode = walker.nextNode();
  while (currentNode) {
    textNodes.push(currentNode as Text);
    currentNode = walker.nextNode();
  }

  if (DEBUG) {
    console.log(`[PageTranslation] Total collected: ${textNodes.length} text nodes`);
  }

  return textNodes;
}

/**
 * Restore original text for all translated elements.
 */
function restoreOriginalText(rootElement: Element): void {
  const translatedElements = rootElement.querySelectorAll(`[${TRANSLATED_ATTR}]`);

  if (DEBUG) {
    console.log(`[PageTranslation] Restoring ${translatedElements.length} translated elements`);
  }

  translatedElements.forEach((element) => {
    const originalText = element.getAttribute(ORIGINAL_TEXT_ATTR);
    if (originalText !== null) {
      // Find the text node and restore
      const walker = document.createTreeWalker(
        element,
        NodeFilter.SHOW_TEXT,
        null
      );

      let textNode = walker.nextNode() as Text;
      if (textNode) {
        textNode.textContent = originalText;
      }

      // Remove translation attributes
      element.removeAttribute(TRANSLATED_ATTR);
      element.removeAttribute(ORIGINAL_TEXT_ATTR);
    }
  });
}

/**
 * Get page slug for translation caching.
 */
function getPageSlug(): string {
  if (typeof window === 'undefined') return 'page';

  // Use pathname as slug (sanitized)
  const pathname = window.location.pathname;
  const slug = pathname
    .replace(/^\/+|\/+$/g, '') // Remove leading/trailing slashes
    .replace(/[^a-z0-9-_]/gi, '-') // Replace invalid chars
    .replace(/-+/g, '-') // Collapse multiple dashes
    || 'home';

  return `page-${slug}`;
}

/**
 * Hook for page-level translation.
 *
 * @param options - Configuration options
 * @param options.rootSelector - CSS selector for root element to translate
 */
export function usePageTranslation(options: {
  rootSelector?: string;
} = {}) {
  const {
    isEnabled,
    language,
    isTranslating,
    setIsTranslating,
    setError,
  } = usePageTranslationContext();

  // Use the entire document body to ensure we catch everything
  const { rootSelector = 'body' } = options;

  // Track if translation has been attempted for current page state
  const lastTranslationKey = useRef<string>('');
  const isTranslatingRef = useRef(false);

  /**
   * Translate the page content.
   */
  const translatePage = useCallback(async () => {
    if (typeof window === 'undefined') return;
    if (isTranslatingRef.current) {
      if (DEBUG) console.log('[PageTranslation] Translation already in progress, skipping');
      return;
    }

    const currentKey = `${window.location.pathname}-${language}-${isEnabled}`;
    if (lastTranslationKey.current === currentKey) {
      if (DEBUG) console.log('[PageTranslation] Already translated this state, skipping');
      return;
    }

    // Find root element
    const rootElement = document.querySelector(rootSelector);
    if (!rootElement) {
      console.warn(`[PageTranslation] Root element not found: ${rootSelector}`);
      return;
    }

    if (DEBUG) {
      console.log('[PageTranslation] ======================');
      console.log('[PageTranslation] Starting translation...');
      console.log('[PageTranslation] Enabled:', isEnabled);
      console.log('[PageTranslation] Language:', language);
      console.log('[PageTranslation] Root selector:', rootSelector);
    }

    try {
      isTranslatingRef.current = true;
      setIsTranslating(true);
      setError(null);

      if (!isEnabled) {
        // Restore original text
        if (DEBUG) console.log('[PageTranslation] Restoring original text...');
        restoreOriginalText(rootElement);
        lastTranslationKey.current = currentKey;
        return;
      }

      // Collect text nodes
      const textNodes = collectTextNodes(rootElement);

      if (textNodes.length === 0) {
        console.warn('[PageTranslation] No text nodes found to translate');
        lastTranslationKey.current = currentKey;
        return;
      }

      // Extract original texts
      const originalTexts = textNodes.map((node) => node.textContent?.trim() || '');

      if (DEBUG) {
        console.log(`[PageTranslation] Collected ${originalTexts.length} text segments`);
        console.log('[PageTranslation] First 5 segments:', originalTexts.slice(0, 5));
        console.log('[PageTranslation] Total characters:', originalTexts.join('').length);
      }

      // Combine texts with separator for batch translation
      const combinedText = originalTexts.join(SEPARATOR);

      if (DEBUG) {
        console.log('[PageTranslation] Combined text length:', combinedText.length);
        console.log('[PageTranslation] Calling translation API...');
      }

      // Call translation API
      const pageSlug = getPageSlug();
      const response = await apiClient.requestTranslation(pageSlug, {
        language,
        content: combinedText,
      });

      if (DEBUG) {
        console.log('[PageTranslation] API response:', response);
      }

      // Handle pending/in-progress response
      if ('status' in response && response.status !== 'completed') {
        if (DEBUG) console.log('[PageTranslation] Translation pending, will poll...');

        // Poll for completion
        let attempts = 0;
        const maxAttempts = 30;  // Increased from 20
        const pollInterval = 2000;  // Decreased from 3000

        const poll = async (): Promise<void> => {
          attempts++;
          if (DEBUG) console.log(`[PageTranslation] Polling attempt ${attempts}/${maxAttempts}`);

          if (attempts > maxAttempts) {
            throw new Error('Translation timeout - please try again');
          }

          const pollResponse = await apiClient.getTranslation(pageSlug, language);

          if ('content' in pollResponse) {
            // Completed
            if (DEBUG) console.log('[PageTranslation] Translation completed!');
            applyTranslation(pollResponse.content, textNodes, originalTexts);
            lastTranslationKey.current = currentKey;
          } else {
            // Still pending, continue polling
            setTimeout(poll, pollInterval);
          }
        };

        setTimeout(poll, pollInterval);
        return;
      }

      // Apply translation immediately if completed
      if ('content' in response) {
        if (DEBUG) console.log('[PageTranslation] Translation completed immediately!');
        applyTranslation(response.content, textNodes, originalTexts);
        lastTranslationKey.current = currentKey;
      }
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Translation failed';
      console.error('[PageTranslation] Translation error:', err);
      setError(errorMessage);

      // Restore original text on error
      restoreOriginalText(rootElement);
    } finally {
      isTranslatingRef.current = false;
      setIsTranslating(false);
    }
  }, [isEnabled, language, rootSelector, setIsTranslating, setError]);

  /**
   * Apply translated content to text nodes.
   */
  const applyTranslation = useCallback((
    translatedContent: string,
    textNodes: Text[],
    originalTexts: string[]
  ) => {
    if (DEBUG) {
      console.log('[PageTranslation] Applying translation...');
      console.log('[PageTranslation] Translated content length:', translatedContent.length);
    }

    // Split translated content
    const translatedTexts = translatedContent.split(SEPARATOR);

    if (DEBUG) {
      console.log(`[PageTranslation] Split into ${translatedTexts.length} segments (expected ${textNodes.length})`);
      console.log('[PageTranslation] First 5 translated:', translatedTexts.slice(0, 5));
    }

    // Apply to each text node
    let appliedCount = 0;
    textNodes.forEach((node, index) => {
      const translatedText = translatedTexts[index];
      const originalText = originalTexts[index];

      if (translatedText && node.parentElement) {
        // Store original text as attribute
        node.parentElement.setAttribute(ORIGINAL_TEXT_ATTR, originalText);
        node.parentElement.setAttribute(TRANSLATED_ATTR, 'true');

        // Update text content
        node.textContent = translatedText;
        appliedCount++;
      }
    });

    if (DEBUG) {
      console.log(`[PageTranslation] Applied translation to ${appliedCount} text nodes`);
      console.log('[PageTranslation] ======================');
    }
  }, []);

  /**
   * Effect to trigger translation when state changes.
   * Uses a longer delay to ensure Docusaurus has fully rendered the page.
   */
  useEffect(() => {
    // Wait for Docusaurus to fully render content
    const timeoutId = setTimeout(() => {
      translatePage();
    }, 300);

    return () => clearTimeout(timeoutId);
  }, [translatePage]);

  /**
   * Effect to handle navigation (Docusaurus-specific).
   */
  useEffect(() => {
    if (typeof window === 'undefined') return;

    // Reset translation key on navigation
    const handleNavigation = () => {
      if (DEBUG) console.log('[PageTranslation] Navigation detected, resetting...');
      lastTranslationKey.current = '';
      // Longer delay to let Docusaurus fully render new content
      setTimeout(() => {
        if (isEnabled) {
          translatePage();
        }
      }, 800);  // Wait for Docusaurus hydration and content load
    };

    // Listen for Docusaurus route changes
    window.addEventListener('popstate', handleNavigation);

    // Listen for custom Docusaurus events (if available)
    if ('docusaurus' in window) {
      // Docusaurus v3 uses history API
      const originalPushState = window.history.pushState;
      const originalReplaceState = window.history.replaceState;

      window.history.pushState = function(...args) {
        originalPushState.apply(this, args);
        handleNavigation();
      };

      window.history.replaceState = function(...args) {
        originalReplaceState.apply(this, args);
        handleNavigation();
      };

      return () => {
        window.history.pushState = originalPushState;
        window.history.replaceState = originalReplaceState;
        window.removeEventListener('popstate', handleNavigation);
      };
    }

    return () => {
      window.removeEventListener('popstate', handleNavigation);
    };
  }, [isEnabled, translatePage]);

  return {
    isTranslating,
    translatePage,
  };
}

export default usePageTranslation;
