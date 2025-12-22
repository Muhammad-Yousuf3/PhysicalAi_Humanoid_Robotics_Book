/**
 * Floating ChatWidget component for RAG-powered Q&A.
 * Integrates with backend /api/chat and /api/chat/selected endpoints.
 * Includes chat history for authenticated users.
 */

import { useState, useRef, useEffect, type KeyboardEvent } from 'react';
import { useChat, type Message } from '../../hooks/useChat';
import { useSelectedText } from '../../hooks/useSelectedText';
import { useChatHistory, type ConversationSummary } from '../../hooks/useChatHistory';
import { useAuthContext } from '../../hooks/useAuth';
import styles from './styles.module.css';

// Icons as inline SVG for simplicity
const ChatIcon = () => (
  <svg viewBox="0 0 24 24" fill="currentColor">
    <path d="M20 2H4c-1.1 0-2 .9-2 2v18l4-4h14c1.1 0 2-.9 2-2V4c0-1.1-.9-2-2-2zm0 14H6l-2 2V4h16v12z" />
  </svg>
);

const CloseIcon = () => (
  <svg viewBox="0 0 24 24" fill="currentColor">
    <path d="M19 6.41L17.59 5 12 10.59 6.41 5 5 6.41 10.59 12 5 17.59 6.41 19 12 13.41 17.59 19 19 17.59 13.41 12z" />
  </svg>
);

const ClearIcon = () => (
  <svg viewBox="0 0 24 24" fill="currentColor">
    <path d="M6 19c0 1.1.9 2 2 2h8c1.1 0 2-.9 2-2V7H6v12zM19 4h-3.5l-1-1h-5l-1 1H5v2h14V4z" />
  </svg>
);

const SendIcon = () => (
  <svg viewBox="0 0 24 24" fill="currentColor">
    <path d="M2.01 21L23 12 2.01 3 2 10l15 2-15 2z" />
  </svg>
);

const BookIcon = () => (
  <svg viewBox="0 0 24 24" fill="currentColor">
    <path d="M18 2H6c-1.1 0-2 .9-2 2v16c0 1.1.9 2 2 2h12c1.1 0 2-.9 2-2V4c0-1.1-.9-2-2-2zM6 4h5v8l-2.5-1.5L6 12V4z" />
  </svg>
);

const HistoryIcon = () => (
  <svg viewBox="0 0 24 24" fill="currentColor">
    <path d="M13 3a9 9 0 0 0-9 9H1l3.89 3.89.07.14L9 12H6c0-3.87 3.13-7 7-7s7 3.13 7 7-3.13 7-7 7c-1.93 0-3.68-.79-4.94-2.06l-1.42 1.42A8.954 8.954 0 0 0 13 21a9 9 0 0 0 0-18zm-1 5v5l4.28 2.54.72-1.21-3.5-2.08V8H12z" />
  </svg>
);

const BackIcon = () => (
  <svg viewBox="0 0 24 24" fill="currentColor">
    <path d="M20 11H7.83l5.59-5.59L12 4l-8 8 8 8 1.41-1.41L7.83 13H20v-2z" />
  </svg>
);

interface MessageBubbleProps {
  message: Message;
}

function MessageBubble({ message }: MessageBubbleProps) {
  const isUser = message.role === 'user';

  return (
    <div
      className={`${styles.message} ${
        isUser ? styles.userMessage : styles.assistantMessage
      }`}
    >
      <div>{message.content}</div>
      {message.sources && message.sources.length > 0 && (
        <div className={styles.sources}>
          <strong>Sources:</strong>
          {message.sources.map((source, idx) => (
            <span key={idx} className={styles.sourceItem}>
              {source.chapter}
              {source.section ? ` - ${source.section}` : ''}
              {source.page ? ` (p.${source.page})` : ''}
            </span>
          ))}
        </div>
      )}
    </div>
  );
}

export default function ChatWidget() {
  const [isOpen, setIsOpen] = useState(false);
  const [inputValue, setInputValue] = useState('');
  const [showHistory, setShowHistory] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLTextAreaElement>(null);

  const {
    messages,
    isLoading,
    error,
    sendMessage,
    sendSelectedTextMessage,
    clearChat,
    setMessagesFromHistory,
    setConversationId,
  } = useChat();

  const { selectedText, hasSelection, clearSelection } = useSelectedText();
  const [useSelection, setUseSelection] = useState(false);

  // Auth context for checking if user is logged in
  let isAuthenticated = false;
  try {
    const auth = useAuthContext();
    isAuthenticated = auth.isAuthenticated;
  } catch {
    // Auth context not available
  }

  // Chat history
  const {
    conversations,
    isLoading: historyLoading,
    fetchConversations,
    fetchConversation,
  } = useChatHistory();

  // Load history when opening history panel
  useEffect(() => {
    if (showHistory && isAuthenticated) {
      fetchConversations();
    }
  }, [showHistory, isAuthenticated, fetchConversations]);

  // Track if user has selected text while chat is open
  useEffect(() => {
    if (hasSelection && isOpen) {
      setUseSelection(true);
    }
  }, [hasSelection, isOpen]);

  // Scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Focus input when chat opens
  useEffect(() => {
    if (isOpen && !showHistory) {
      inputRef.current?.focus();
    }
  }, [isOpen, showHistory]);

  const handleSend = async () => {
    if (!inputValue.trim() || isLoading) return;

    const message = inputValue;
    setInputValue('');

    if (useSelection && selectedText) {
      await sendSelectedTextMessage(message, selectedText);
      setUseSelection(false);
      clearSelection();
    } else {
      await sendMessage(message);
    }
  };

  const handleKeyDown = (e: KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  const handleClearSelection = () => {
    setUseSelection(false);
    clearSelection();
  };

  const toggleChat = () => {
    setIsOpen((prev) => !prev);
    if (!isOpen) {
      setShowHistory(false);
    }
  };

  const toggleHistory = () => {
    setShowHistory((prev) => !prev);
  };

  const loadConversation = async (conv: ConversationSummary) => {
    const detail = await fetchConversation(conv.id);
    if (detail) {
      const loadedMessages: Message[] = detail.messages.map((msg: any) => ({
        id: msg.id,
        role: msg.role as 'user' | 'assistant',
        content: msg.content,
        timestamp: new Date(msg.created_at),
      }));
      setMessagesFromHistory(loadedMessages);
      setConversationId(conv.id);
      setShowHistory(false);
    }
  };

  return (
    <div className={styles.chatWidget}>
      {isOpen && (
        <div className={styles.chatWindow}>
          {/* Header */}
          <div className={styles.header}>
            {showHistory ? (
              <button
                className={styles.headerButton}
                onClick={toggleHistory}
                title="Back to chat"
                aria-label="Back to chat"
              >
                <BackIcon />
              </button>
            ) : null}
            <h3 className={styles.headerTitle}>
              {showHistory ? 'Chat History' : 'Book Assistant'}
            </h3>
            <div className={styles.headerActions}>
              {!showHistory && isAuthenticated && (
                <button
                  className={styles.headerButton}
                  onClick={toggleHistory}
                  title="View history"
                  aria-label="View chat history"
                >
                  <HistoryIcon />
                </button>
              )}
              {!showHistory && (
                <button
                  className={styles.headerButton}
                  onClick={clearChat}
                  title="New chat"
                  aria-label="Start new chat"
                >
                  <ClearIcon />
                </button>
              )}
              <button
                className={styles.headerButton}
                onClick={toggleChat}
                title="Close"
                aria-label="Close chat"
              >
                <CloseIcon />
              </button>
            </div>
          </div>

          {/* Messages or History */}
          <div className={styles.messagesContainer}>
            {showHistory ? (
              // History Panel
              <div className={styles.historyPanel}>
                {historyLoading ? (
                  <div className={styles.loadingIndicator}>
                    <div className={styles.loadingDots}>
                      <span></span>
                      <span></span>
                      <span></span>
                    </div>
                    <span>Loading history...</span>
                  </div>
                ) : conversations.length === 0 ? (
                  <div className={styles.emptyState}>
                    <HistoryIcon />
                    <p>No conversation history yet.</p>
                    <p style={{ marginTop: 8, fontSize: 12 }}>
                      Your conversations will appear here.
                    </p>
                  </div>
                ) : (
                  conversations.map((conv) => (
                    <button
                      key={conv.id}
                      className={styles.historyItem}
                      onClick={() => loadConversation(conv)}
                    >
                      <div className={styles.historyItemHeader}>
                        <span className={styles.historyItemMode}>
                          {conv.mode === 'selected_text' ? 'Selected Text' : 'Full Book'}
                        </span>
                        <span className={styles.historyItemCount}>
                          {conv.message_count} messages
                        </span>
                      </div>
                      <p className={styles.historyItemPreview}>
                        {conv.last_message_preview || 'No messages'}
                      </p>
                      <span className={styles.historyItemDate}>
                        {new Date(conv.updated_at).toLocaleDateString()}
                      </span>
                    </button>
                  ))
                )}
              </div>
            ) : messages.length === 0 ? (
              <div className={styles.emptyState}>
                <BookIcon />
                <p>
                  Ask me anything about the book! I'll provide answers grounded
                  in the content with source citations.
                </p>
                <p style={{ marginTop: 8, fontSize: 12 }}>
                  Tip: Select text on the page to ask about specific content.
                </p>
              </div>
            ) : (
              <>
                {messages.map((msg) => (
                  <MessageBubble key={msg.id} message={msg} />
                ))}
                {isLoading && (
                  <div className={styles.loadingIndicator}>
                    <div className={styles.loadingDots}>
                      <span></span>
                      <span></span>
                      <span></span>
                    </div>
                    <span>Thinking...</span>
                  </div>
                )}
                <div ref={messagesEndRef} />
              </>
            )}
          </div>

          {/* Error */}
          {error && <div className={styles.errorMessage}>{error}</div>}

          {/* Input */}
          <div className={styles.inputArea}>
            {useSelection && selectedText && (
              <div className={styles.selectedTextBanner}>
                <span>
                  Asking about: "{selectedText.slice(0, 50)}
                  {selectedText.length > 50 ? '...' : ''}"
                </span>
                <button onClick={handleClearSelection}>Clear</button>
              </div>
            )}
            <div className={styles.inputWrapper}>
              <textarea
                ref={inputRef}
                className={styles.textInput}
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                onKeyDown={handleKeyDown}
                placeholder={
                  useSelection && selectedText
                    ? 'Ask about the selected text...'
                    : 'Ask a question about the book...'
                }
                rows={1}
                disabled={isLoading}
              />
              <button
                className={styles.sendButton}
                onClick={handleSend}
                disabled={!inputValue.trim() || isLoading}
                aria-label="Send message"
              >
                <SendIcon />
              </button>
            </div>
          </div>
        </div>
      )}

      {/* Toggle button */}
      <button
        className={styles.toggleButton}
        onClick={toggleChat}
        aria-label={isOpen ? 'Close chat' : 'Open chat'}
      >
        {isOpen ? <CloseIcon /> : <ChatIcon />}
      </button>
    </div>
  );
}
