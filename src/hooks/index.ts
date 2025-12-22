/**
 * Hook exports for RAG chatbot integration.
 */

export { useChat, type Message, type UseChatReturn } from './useChat';
export { useSelectedText, type UseSelectedTextReturn } from './useSelectedText';
export { useTranslate, type UseTranslateReturn } from './useTranslate';
export { useAuth, useAuthContext, AuthContext, type User, type AuthState, type UseAuthReturn } from './useAuth';
export { useChatHistory, type ConversationSummary, type ConversationDetail, type UseChatHistoryReturn } from './useChatHistory';
