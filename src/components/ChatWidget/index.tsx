/**
 * Floating ChatWidget component for RAG-powered Q&A.
 * Integrates with backend /api/chat and /api/chat/selected endpoints.
 */

import { useState, useRef, useEffect, type KeyboardEvent } from 'react';
import { useChat, type Message } from '../../hooks/useChat';
import { useSelectedText } from '../../hooks/useSelectedText';
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
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLTextAreaElement>(null);

  const {
    messages,
    isLoading,
    error,
    sendMessage,
    sendSelectedTextMessage,
    clearChat,
  } = useChat();

  const { selectedText, hasSelection, clearSelection } = useSelectedText();
  const [useSelection, setUseSelection] = useState(false);

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
    if (isOpen) {
      inputRef.current?.focus();
    }
  }, [isOpen]);

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
  };

  return (
    <div className={styles.chatWidget}>
      {isOpen && (
        <div className={styles.chatWindow}>
          {/* Header */}
          <div className={styles.header}>
            <h3 className={styles.headerTitle}>Book Assistant</h3>
            <div className={styles.headerActions}>
              <button
                className={styles.headerButton}
                onClick={clearChat}
                title="New chat"
                aria-label="Start new chat"
              >
                <ClearIcon />
              </button>
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

          {/* Messages */}
          <div className={styles.messagesContainer}>
            {messages.length === 0 ? (
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
