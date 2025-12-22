/**
 * API utility for communicating with the RAG chatbot backend.
 * Uses environment variable for backend URL configuration.
 */

import type {
  ChatRequest,
  ChatResponse,
  SelectedTextRequest,
  TranslationRequest,
  TranslationResult,
} from '../types/api';

// Default API URL - can be overridden via docusaurus.config.ts customFields or runtime config
const DEFAULT_API_URL = 'http://127.0.0.1:8000/api';

// Get API URL from Docusaurus custom fields, runtime config, or fallback
function getApiUrl(): string {
  if (typeof window !== 'undefined') {
    // Check for runtime config (injected via script)
    if ((window as any).RAG_CHATBOT_API_URL) {
      const url = (window as any).RAG_CHATBOT_API_URL;
      // Ensure it ends with /api if not present, assuming base URL is root
      return url.endsWith('/api') ? url : `${url}/api`;
    }

    // Check for custom field set in docusaurus.config.ts (client-side only)
    const docusaurus = (window as any).__DOCUSAURUS__;
    if (docusaurus?.siteConfig?.customFields?.apiUrl) {
      return docusaurus.siteConfig.customFields.apiUrl as string;
    }
  }
  return DEFAULT_API_URL;
}

class ApiClient {
  private baseUrl: string;

  constructor() {
    this.baseUrl = getApiUrl();
  }

  private getAuthToken(): string | null {
    if (typeof window === 'undefined') return null;
    return localStorage.getItem('auth_token');
  }

  private async request<T>(
    endpoint: string,
    options: RequestInit = {},
    requireAuth = false
  ): Promise<T> {
    // Ensure base URL is fresh in case of hydration mismatches or late loading
    this.baseUrl = getApiUrl(); 
    const url = `${this.baseUrl}${endpoint}`;
    
    const headers: Record<string, string> = {
      ...((options.headers as Record<string, string>) || {}),
    };

    // Add Content-Type for POST/PUT if not present
    if (options.method && ['POST', 'PUT', 'PATCH'].includes(options.method.toUpperCase())) {
        if (!headers['Content-Type']) {
            headers['Content-Type'] = 'application/json';
        }
    }

    // Add auth token if required
    const token = this.getAuthToken();
    if (requireAuth && token) {
      headers['Authorization'] = `Bearer ${token}`;
    }

    const response = await fetch(url, {
      ...options,
      headers,
    });

    // Handle JSON parsing and errors
    let data;
    const contentType = response.headers.get('content-type');
    if (contentType && contentType.includes('application/json')) {
        data = await response.json().catch(() => ({}));
    } else {
        data = { text: await response.text() };
    }

    if (!response.ok) {
      const errorMessage = data.detail || data.message || `API error: ${response.status}`;
      throw new Error(errorMessage);
    }

    return data as T;
  }

  /**
   * Send a chat message and get a book-grounded response.
   * Method: POST /api/chat
   */
  async chat(data: ChatRequest): Promise<ChatResponse> {
    return this.request<ChatResponse>('/chat', {
      method: 'POST',
      body: JSON.stringify(data),
    });
  }

  /**
   * Send a message about selected text.
   * Method: POST /api/chat/selected
   */
  async chatSelected(data: SelectedTextRequest): Promise<ChatResponse> {
    return this.request<ChatResponse>('/chat/selected', {
      method: 'POST',
      body: JSON.stringify(data),
    });
  }

  /**
   * Get translation for a chapter.
   * Method: GET /api/translate/{slug}
   */
  async getTranslation(
    chapterSlug: string,
    language = 'ur'
  ): Promise<TranslationResult> {
    return this.request<TranslationResult>(
      `/translate/${encodeURIComponent(chapterSlug)}?language=${language}`,
      { method: 'GET' }
    );
  }

  /**
   * Request translation for a chapter (trigger generation).
   * Method: POST /api/translate/{slug}
   */
  async requestTranslation(
    chapterSlug: string,
    data: TranslationRequest
  ): Promise<TranslationResult> {
    return this.request<TranslationResult>(
      `/translate/${encodeURIComponent(chapterSlug)}`,
      {
        method: 'POST',
        body: JSON.stringify(data),
      }
    );
  }

  /**
   * Get list of conversations (requires auth).
   * Method: GET /api/chat/conversations
   */
  async getConversations(limit = 20, offset = 0): Promise<{
    conversations: any[];
    total: number;
  }> {
    return this.request(
      `/chat/conversations?limit=${limit}&offset=${offset}`,
      { method: 'GET' },
      true
    );
  }

  /**
   * Get a specific conversation with messages (requires auth).
   * Method: GET /api/chat/conversations/{id}
   */
  async getConversation(id: string): Promise<any> {
    return this.request(`/chat/conversations/${id}`, { method: 'GET' }, true);
  }
}

// Singleton instance
export const apiClient = new ApiClient();