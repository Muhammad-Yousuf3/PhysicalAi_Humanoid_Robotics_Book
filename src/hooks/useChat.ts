/**
 * Hook for chat functionality with the RAG backend.
 */

import { useState, useCallback } from 'react';
import { apiClient } from '../utils/api';
import type { ChatResponse, SourceReference } from '../types/api';

export interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  sources?: SourceReference[];
  timestamp: Date;
}

export interface UseChatReturn {
  messages: Message[];
  isLoading: boolean;
  error: string | null;
  conversationId: string | null;
  sendMessage: (message: string) => Promise<void>;
  sendSelectedTextMessage: (message: string, selectedText: string) => Promise<void>;
  clearChat: () => void;
  setMessagesFromHistory: (msgs: Message[]) => void;
  setConversationId: (id: string | null) => void;
}

export function useChat(): UseChatReturn {
  const [messages, setMessages] = useState<Message[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [conversationId, setConversationId] = useState<string | null>(null);

  const generateId = () => `msg-${Date.now()}-${Math.random().toString(36).slice(2, 9)}`;

  const sendMessage = useCallback(async (message: string) => {
    if (!message.trim()) return;

    const userMessage: Message = {
      id: generateId(),
      role: 'user',
      content: message,
      timestamp: new Date(),
    };

    setMessages((prev) => [...prev, userMessage]);
    setIsLoading(true);
    setError(null);

    try {
      const response: ChatResponse = await apiClient.chat({
        message,
        conversation_id: conversationId,
      });

      const assistantMessage: Message = {
        id: generateId(),
        role: 'assistant',
        content: response.message,
        sources: response.sources,
        timestamp: new Date(),
      };

      setMessages((prev) => [...prev, assistantMessage]);
      setConversationId(response.conversation_id);
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Failed to send message';
      setError(errorMessage);
    } finally {
      setIsLoading(false);
    }
  }, [conversationId]);

  const sendSelectedTextMessage = useCallback(
    async (message: string, selectedText: string) => {
      if (!message.trim() || !selectedText.trim()) return;

      const userMessage: Message = {
        id: generateId(),
        role: 'user',
        content: `[About selected text]\n${message}`,
        timestamp: new Date(),
      };

      setMessages((prev) => [...prev, userMessage]);
      setIsLoading(true);
      setError(null);

      try {
        const response: ChatResponse = await apiClient.chatSelected({
          message,
          selected_text: selectedText,
          conversation_id: conversationId,
        });

        const assistantMessage: Message = {
          id: generateId(),
          role: 'assistant',
          content: response.message,
          timestamp: new Date(),
        };

        setMessages((prev) => [...prev, assistantMessage]);
        setConversationId(response.conversation_id);
      } catch (err) {
        const errorMessage = err instanceof Error ? err.message : 'Failed to send message';
        setError(errorMessage);
      } finally {
        setIsLoading(false);
      }
    },
    [conversationId]
  );

  const clearChat = useCallback(() => {
    setMessages([]);
    setConversationId(null);
    setError(null);
  }, []);

  const setMessagesFromHistory = useCallback((msgs: Message[]) => {
    setMessages(msgs);
  }, []);

  const updateConversationId = useCallback((id: string | null) => {
    setConversationId(id);
  }, []);

  return {
    messages,
    isLoading,
    error,
    conversationId,
    sendMessage,
    sendSelectedTextMessage,
    clearChat,
    setMessagesFromHistory,
    setConversationId: updateConversationId,
  };
}
