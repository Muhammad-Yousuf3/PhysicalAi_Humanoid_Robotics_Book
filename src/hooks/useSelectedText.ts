/**
 * Hook for detecting and managing selected text in the document.
 * SSR-safe: Only attaches event listeners on client-side.
 */

import { useState, useEffect, useCallback } from 'react';

export interface UseSelectedTextReturn {
  selectedText: string;
  hasSelection: boolean;
  selectionPosition: { x: number; y: number } | null;
  clearSelection: () => void;
}

export function useSelectedText(): UseSelectedTextReturn {
  const [selectedText, setSelectedText] = useState('');
  const [selectionPosition, setSelectionPosition] = useState<{ x: number; y: number } | null>(null);

  const handleSelectionChange = useCallback(() => {
    if (typeof window === 'undefined') return;

    const selection = window.getSelection();
    const text = selection?.toString().trim() || '';

    if (text.length >= 10) {
      setSelectedText(text);

      // Get position for potential popup
      if (selection && selection.rangeCount > 0) {
        const range = selection.getRangeAt(0);
        const rect = range.getBoundingClientRect();
        setSelectionPosition({
          x: rect.left + rect.width / 2,
          y: rect.top - 10,
        });
      }
    } else {
      setSelectedText('');
      setSelectionPosition(null);
    }
  }, []);

  const clearSelection = useCallback(() => {
    setSelectedText('');
    setSelectionPosition(null);
    if (typeof window !== 'undefined') {
      window.getSelection()?.removeAllRanges();
    }
  }, []);

  useEffect(() => {
    // Skip on server-side
    if (typeof window === 'undefined') return;

    // Use mouseup for better selection detection
    const handleMouseUp = () => {
      // Small delay to ensure selection is complete
      setTimeout(handleSelectionChange, 10);
    };

    // Also handle keyboard selection
    const handleKeyUp = (e: KeyboardEvent) => {
      if (e.shiftKey) {
        setTimeout(handleSelectionChange, 10);
      }
    };

    document.addEventListener('mouseup', handleMouseUp);
    document.addEventListener('keyup', handleKeyUp);

    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
      document.removeEventListener('keyup', handleKeyUp);
    };
  }, [handleSelectionChange]);

  return {
    selectedText,
    hasSelection: selectedText.length >= 10,
    selectionPosition,
    clearSelection,
  };
}
