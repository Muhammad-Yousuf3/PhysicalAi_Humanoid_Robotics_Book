/**
 * API types for RAG chatbot integration.
 * Matches the backend FastAPI schemas.
 */

// Chat types
export interface SourceReference {
  chapter: string;
  section?: string | null;
  page?: number | null;
  relevance: number;
}

export interface ChatRequest {
  message: string;
  conversation_id?: string | null;
}

export interface SelectedTextRequest {
  message: string;
  selected_text: string;
  conversation_id?: string | null;
}

export interface ChatResponse {
  message: string;
  conversation_id: string;
  sources: SourceReference[];
}

// Translation types
export interface TranslationRequest {
  language: string;
  content: string;
}

export interface TranslationResponse {
  chapter_slug: string;
  language: string;
  content: string;
  created_at: string;
}

export interface TranslationPendingResponse {
  chapter_slug: string;
  language: string;
  status: 'pending' | 'in_progress';
  estimated_seconds?: number | null;
}

export type TranslationResult = TranslationResponse | TranslationPendingResponse;

// Type guard for translation response
export function isTranslationComplete(
  result: TranslationResult
): result is TranslationResponse {
  return 'content' in result && 'created_at' in result;
}
