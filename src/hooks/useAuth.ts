/**
 * Hook for authentication with the RAG backend.
 * Connects to /api/auth endpoints.
 */

import { useState, useCallback, useEffect, createContext, useContext } from 'react';

// Default API URL - can be overridden via docusaurus.config.ts customFields
const DEFAULT_API_URL = 'http://localhost:8000/api';

// Get API URL from Docusaurus custom fields or fallback
function getApiUrl(): string {
  if (typeof window !== 'undefined') {
    const docusaurus = (window as any).__DOCUSAURUS__;
    if (docusaurus?.siteConfig?.customFields?.apiUrl) {
      return docusaurus.siteConfig.customFields.apiUrl as string;
    }
  }
  return DEFAULT_API_URL;
}

export interface User {
  id: string;
  email: string;
  name?: string;
}

export interface AuthState {
  user: User | null;
  isAuthenticated: boolean;
  isLoading: boolean;
  error: string | null;
}

export interface UseAuthReturn extends AuthState {
  signIn: (email: string, password: string) => Promise<boolean>;
  signUp: (email: string, password: string, name?: string) => Promise<boolean>;
  signOut: () => void;
  clearError: () => void;
}

const TOKEN_KEY = 'auth_token';

// Helper to safely access localStorage (SSR-safe)
function getStoredToken(): string | null {
  if (typeof window === 'undefined') return null;
  return localStorage.getItem(TOKEN_KEY);
}

function setStoredToken(token: string): void {
  if (typeof window !== 'undefined') {
    localStorage.setItem(TOKEN_KEY, token);
  }
}

function removeStoredToken(): void {
  if (typeof window !== 'undefined') {
    localStorage.removeItem(TOKEN_KEY);
  }
}

export function useAuth(): UseAuthReturn {
  const [user, setUser] = useState<User | null>(null);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const baseUrl = getApiUrl();

  // Check for existing token on mount (client-side only)
  useEffect(() => {
    const token = getStoredToken();
    if (token) {
      validateToken(token);
    }
  }, []);

  const validateToken = async (token: string) => {
    try {
      const response = await fetch(`${baseUrl}/auth/me`, {
        headers: {
          Authorization: `Bearer ${token}`,
        },
      });

      if (response.ok) {
        const userData = await response.json();
        setUser(userData);
      } else {
        removeStoredToken();
      }
    } catch {
      removeStoredToken();
    }
  };

  const signIn = useCallback(async (email: string, password: string): Promise<boolean> => {
    setIsLoading(true);
    setError(null);

    try {
      const response = await fetch(`${baseUrl}/auth/login`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ email, password }),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || 'Invalid email or password');
      }

      const data = await response.json();
      setStoredToken(data.access_token);
      setUser(data.user);
      return true;
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Sign in failed';
      setError(errorMessage);
      return false;
    } finally {
      setIsLoading(false);
    }
  }, [baseUrl]);

  const signUp = useCallback(async (email: string, password: string, name?: string): Promise<boolean> => {
    setIsLoading(true);
    setError(null);

    try {
      const response = await fetch(`${baseUrl}/auth/register`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ email, password, name }),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || 'Registration failed');
      }

      const data = await response.json();
      setStoredToken(data.access_token);
      setUser(data.user);
      return true;
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Sign up failed';
      setError(errorMessage);
      return false;
    } finally {
      setIsLoading(false);
    }
  }, [baseUrl]);

  const signOut = useCallback(() => {
    removeStoredToken();
    setUser(null);
    setError(null);
  }, []);

  const clearError = useCallback(() => {
    setError(null);
  }, []);

  return {
    user,
    isAuthenticated: !!user,
    isLoading,
    error,
    signIn,
    signUp,
    signOut,
    clearError,
  };
}

// Auth Context for sharing state across components
interface AuthContextType extends UseAuthReturn {
  openAuthModal: (mode: 'signin' | 'signup') => void;
  closeAuthModal: () => void;
  authModalMode: 'signin' | 'signup' | null;
}

export const AuthContext = createContext<AuthContextType | null>(null);

export function useAuthContext(): AuthContextType {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuthContext must be used within AuthProvider');
  }
  return context;
}
