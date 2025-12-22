/**
 * AuthModal component for Sign In / Sign Up forms.
 */

import { useState, type FormEvent } from 'react';
import { useAuthContext } from '../../hooks/useAuth';
import styles from './styles.module.css';

const CloseIcon = () => (
  <svg viewBox="0 0 24 24" fill="currentColor">
    <path d="M19 6.41L17.59 5 12 10.59 6.41 5 5 6.41 10.59 12 5 17.59 6.41 19 12 13.41 17.59 19 19 17.59 13.41 12z" />
  </svg>
);

export default function AuthModal() {
  const {
    authModalMode,
    closeAuthModal,
    openAuthModal,
    signIn,
    signUp,
    isLoading,
    error,
    clearError,
  } = useAuthContext();

  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [name, setName] = useState('');

  if (!authModalMode) return null;

  const isSignIn = authModalMode === 'signin';

  const handleSubmit = async (e: FormEvent) => {
    e.preventDefault();
    clearError();

    let success: boolean;
    if (isSignIn) {
      success = await signIn(email, password);
    } else {
      success = await signUp(email, password, name || undefined);
    }

    if (success) {
      setEmail('');
      setPassword('');
      setName('');
      closeAuthModal();
    }
  };

  const handleSwitch = () => {
    clearError();
    openAuthModal(isSignIn ? 'signup' : 'signin');
  };

  const handleOverlayClick = (e: React.MouseEvent) => {
    if (e.target === e.currentTarget) {
      closeAuthModal();
    }
  };

  return (
    <div className={styles.overlay} onClick={handleOverlayClick}>
      <div className={styles.modal}>
        <div className={styles.header}>
          <h2 className={styles.title}>
            {isSignIn ? 'Sign In' : 'Create Account'}
          </h2>
          <button
            className={styles.closeButton}
            onClick={closeAuthModal}
            aria-label="Close"
          >
            <CloseIcon />
          </button>
        </div>

        <div className={styles.content}>
          <form className={styles.form} onSubmit={handleSubmit}>
            {!isSignIn && (
              <div className={styles.field}>
                <label className={styles.label} htmlFor="name">
                  Name (optional)
                </label>
                <input
                  id="name"
                  type="text"
                  className={styles.input}
                  value={name}
                  onChange={(e) => setName(e.target.value)}
                  placeholder="Your name"
                  autoComplete="name"
                />
              </div>
            )}

            <div className={styles.field}>
              <label className={styles.label} htmlFor="email">
                Email
              </label>
              <input
                id="email"
                type="email"
                className={styles.input}
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                placeholder="you@example.com"
                required
                autoComplete="email"
              />
            </div>

            <div className={styles.field}>
              <label className={styles.label} htmlFor="password">
                Password
              </label>
              <input
                id="password"
                type="password"
                className={styles.input}
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                placeholder="Enter password"
                required
                minLength={6}
                autoComplete={isSignIn ? 'current-password' : 'new-password'}
              />
            </div>

            {error && <div className={styles.error}>{error}</div>}

            <button
              type="submit"
              className={styles.submitButton}
              disabled={isLoading}
            >
              {isLoading
                ? 'Please wait...'
                : isSignIn
                ? 'Sign In'
                : 'Create Account'}
            </button>
          </form>
        </div>

        <div className={styles.footer}>
          <span className={styles.switchText}>
            {isSignIn ? "Don't have an account?" : 'Already have an account?'}
          </span>
          <button className={styles.switchButton} onClick={handleSwitch}>
            {isSignIn ? 'Sign Up' : 'Sign In'}
          </button>
        </div>
      </div>
    </div>
  );
}
