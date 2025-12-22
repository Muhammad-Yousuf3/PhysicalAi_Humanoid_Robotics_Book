/**
 * Hook for managing chat history.
 * Connects to /api/chat/conversations endpoints.
 */

import { useState, useCallback } from 'react';
import { apiClient } from '../utils/api';

export interface ConversationSummary {
  id: string;
  mode: string;
  message_count: number;
  last_message_preview: string | null;
  created_at: string;
  updated_at: string;
}

export interface ConversationMessage {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  created_at: string;
}

export interface ConversationDetail {
  id: string;
  user_id: string | null;
  mode: string;
  selected_text: string | null;
  messages: ConversationMessage[];
  created_at: string;
  updated_at: string;
}

export interface UseChatHistoryReturn {
  conversations: ConversationSummary[];
  currentConversation: ConversationDetail | null;
  isLoading: boolean;
  error: string | null;
  total: number;
  fetchConversations: (limit?: number, offset?: number) => Promise<void>;
  fetchConversation: (id: string) => Promise<ConversationDetail | null>;
  clearHistory: () => void;
}

export function useChatHistory(): UseChatHistoryReturn {
  const [conversations, setConversations] = useState<ConversationSummary[]>([]);
  const [currentConversation, setCurrentConversation] = useState<ConversationDetail | null>(null);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [total, setTotal] = useState(0);

  const fetchConversations = useCallback(async (limit = 20, offset = 0) => {
    setIsLoading(true);
    setError(null);

    try {
      const response = await apiClient.getConversations(limit, offset);
      setConversations(response.conversations);
      setTotal(response.total);
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Failed to load conversations';
      setError(errorMessage);
    } finally {
      setIsLoading(false);
    }
  }, []);

  const fetchConversation = useCallback(async (id: string): Promise<ConversationDetail | null> => {
    setIsLoading(true);
    setError(null);

    try {
      const conversation = await apiClient.getConversation(id);
      setCurrentConversation(conversation);
      return conversation;
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Failed to load conversation';
      setError(errorMessage);
      return null;
    } finally {
      setIsLoading(false);
    }
  }, []);

  const clearHistory = useCallback(() => {
    setConversations([]);
    setCurrentConversation(null);
    setTotal(0);
    setError(null);
  }, []);

  return {
    conversations,
    currentConversation,
    isLoading,
    error,
    total,
    fetchConversations,
    fetchConversation,
    clearHistory,
  };
}
