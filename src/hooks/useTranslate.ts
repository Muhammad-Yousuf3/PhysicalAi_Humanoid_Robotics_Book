/**
 * Hook for translation functionality.
 */

import { useState, useCallback } from 'react';
import { apiClient } from '../utils/api';
import type { TranslationResult } from '../types/api';
import { isTranslationComplete } from '../types/api';

export interface UseTranslateReturn {
  translatedContent: string | null;
  isLoading: boolean;
  isPending: boolean;
  error: string | null;
  estimatedSeconds: number | null;
  requestTranslation: (chapterSlug: string, content: string) => Promise<void>;
  checkTranslation: (chapterSlug: string) => Promise<void>;
  clearTranslation: () => void;
}

export function useTranslate(): UseTranslateReturn {
  const [translatedContent, setTranslatedContent] = useState<string | null>(null);
  const [isLoading, setIsLoading] = useState(false);
  const [isPending, setIsPending] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [estimatedSeconds, setEstimatedSeconds] = useState<number | null>(null);

  const handleResult = useCallback((result: TranslationResult) => {
    if (isTranslationComplete(result)) {
      setTranslatedContent(result.content);
      setIsPending(false);
      setEstimatedSeconds(null);
    } else {
      setIsPending(true);
      setEstimatedSeconds(result.estimated_seconds || null);
    }
  }, []);

  const requestTranslation = useCallback(async (chapterSlug: string, content: string) => {
    setIsLoading(true);
    setError(null);

    try {
      // First check if translation already exists
      try {
        const existing = await apiClient.getTranslation(chapterSlug);
        handleResult(existing);
        return;
      } catch {
        // Not found, request new translation
      }

      const result = await apiClient.requestTranslation(chapterSlug, {
        language: 'ur',
        content,
      });

      handleResult(result);

      // If pending, poll for completion
      if (!isTranslationComplete(result)) {
        pollForCompletion(chapterSlug);
      }
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Translation failed';
      setError(errorMessage);
    } finally {
      setIsLoading(false);
    }
  }, [handleResult]);

  const pollForCompletion = useCallback(async (chapterSlug: string) => {
    const maxAttempts = 30; // Max 5 minutes with 10s intervals
    let attempts = 0;

    const poll = async () => {
      if (attempts >= maxAttempts) {
        setError('Translation timed out');
        setIsPending(false);
        return;
      }

      try {
        const result = await apiClient.getTranslation(chapterSlug);
        handleResult(result);

        if (!isTranslationComplete(result)) {
          attempts++;
          setTimeout(poll, 10000); // Poll every 10 seconds
        }
      } catch {
        attempts++;
        setTimeout(poll, 10000);
      }
    };

    setTimeout(poll, 5000); // Start polling after 5 seconds
  }, [handleResult]);

  const checkTranslation = useCallback(async (chapterSlug: string) => {
    setIsLoading(true);
    setError(null);

    try {
      const result = await apiClient.getTranslation(chapterSlug);
      handleResult(result);
    } catch (err) {
      // Not found is not an error - just means no translation exists
      if (err instanceof Error && err.message.includes('404')) {
        setTranslatedContent(null);
      } else {
        const errorMessage = err instanceof Error ? err.message : 'Failed to check translation';
        setError(errorMessage);
      }
    } finally {
      setIsLoading(false);
    }
  }, [handleResult]);

  const clearTranslation = useCallback(() => {
    setTranslatedContent(null);
    setIsPending(false);
    setError(null);
    setEstimatedSeconds(null);
  }, []);

  return {
    translatedContent,
    isLoading,
    isPending,
    error,
    estimatedSeconds,
    requestTranslation,
    checkTranslation,
    clearTranslation,
  };
}
