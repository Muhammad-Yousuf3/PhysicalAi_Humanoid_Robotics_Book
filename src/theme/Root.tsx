/**
 * Custom Root component for Docusaurus.
 * Wraps the entire app with providers.
 * Uses BrowserOnly for SSR-unsafe components.
 */

import { useState, type ReactNode, useMemo } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import { AuthContext, useAuth } from '../hooks/useAuth';
import { PageTranslationProvider } from '../contexts/PageTranslationContext';
import { usePageTranslation } from '../hooks/usePageTranslation';

interface Props {
  children: ReactNode;
}

/**
 * Inner component that uses the page translation hook.
 */
function PageTranslationHandler({ children }: { children: ReactNode }) {
  // This hook handles automatic translation when enabled
  // Using 'body' to translate the ENTIRE page
  usePageTranslation({ rootSelector: 'body' });
  return <>{children}</>;
}

export default function Root({ children }: Props): ReactNode {
  const auth = useAuth();
  const [authModalMode, setAuthModalMode] = useState<'signin' | 'signup' | null>(null);

  const authContextValue = useMemo(
    () => ({
      ...auth,
      authModalMode,
      openAuthModal: (mode: 'signin' | 'signup') => setAuthModalMode(mode),
      closeAuthModal: () => setAuthModalMode(null),
    }),
    [auth, authModalMode]
  );

  return (
    <PageTranslationProvider defaultLanguage="ur">
      <PageTranslationHandler>
        <AuthContext.Provider value={authContextValue}>
          {children}
          <BrowserOnly fallback={null}>
            {() => {
              const AuthModal = require('../components/AuthModal').default;
              return <AuthModal />;
            }}
          </BrowserOnly>
        </AuthContext.Provider>
      </PageTranslationHandler>
    </PageTranslationProvider>
  );
}
