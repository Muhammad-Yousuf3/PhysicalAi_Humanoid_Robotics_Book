/**
 * NavbarTranslateButton - Global translation toggle button for navbar.
 *
 * Features:
 * - Toggles full-page translation on/off
 * - Shows loading state during translation
 * - Displays error messages gracefully
 * - Persists state across navigation
 * - Accessible and production-ready
 */

import React from 'react';
import { usePageTranslationContext } from '../../contexts/PageTranslationContext';
import './styles.css';

export interface NavbarTranslateButtonProps {
  /**
   * Button label when translation is off.
   */
  label?: string;

  /**
   * Button label when translation is on.
   */
  activeLabel?: string;

  /**
   * Whether to show text label (false = icon only).
   */
  showLabel?: boolean;

  /**
   * Custom CSS class.
   */
  className?: string;

  /**
   * Button variant style.
   */
  variant?: 'default' | 'minimal' | 'outline';
}

/**
 * Loading spinner component.
 */
const LoadingSpinner: React.FC<{ size?: number }> = ({ size = 16 }) => (
  <svg
    width={size}
    height={size}
    viewBox="0 0 24 24"
    fill="none"
    className="translate-button__spinner"
  >
    <circle
      cx="12"
      cy="12"
      r="10"
      stroke="currentColor"
      strokeWidth="3"
      strokeLinecap="round"
      strokeDasharray="32 32"
    />
  </svg>
);

/**
 * Translation icon component - Google Translate style.
 */
const TranslateIcon: React.FC<{ size?: number; active?: boolean }> = ({
  size = 20,
  active = false,
}) => (
  <svg
    width={size}
    height={size}
    viewBox="0 0 24 24"
    fill="currentColor"
    className={`translate-button__icon ${active ? 'translate-button__icon--active' : ''}`}
  >
    <path d="M12.87 15.07l-2.54-2.51.03-.03c1.74-1.94 2.98-4.17 3.71-6.53H17V4h-7V2H8v2H1v1.99h11.17C11.5 7.92 10.44 9.75 9 11.35 8.07 10.32 7.3 9.19 6.69 8h-2c.73 1.63 1.73 3.17 2.98 4.56l-5.09 5.02L4 19l5-5 3.11 3.11.76-2.04zM18.5 10h-2L12 22h2l1.12-3h4.75L21 22h2l-4.5-12zm-2.62 7l1.62-4.33L19.12 17h-3.24z" />
  </svg>
);

/**
 * Navbar translate button component.
 */
export const NavbarTranslateButton: React.FC<NavbarTranslateButtonProps> = ({
  label = 'اردو',
  activeLabel = 'English',
  showLabel = true,
  className = '',
  variant = 'default',
}) => {
  const {
    isEnabled,
    isTranslating,
    error,
    toggleTranslation,
  } = usePageTranslationContext();

  const buttonLabel = isEnabled ? activeLabel : label;
  const variantClass = `translate-button--${variant}`;
  const activeClass = isEnabled ? 'translate-button--active' : '';
  const loadingClass = isTranslating ? 'translate-button--loading' : '';

  return (
    <div className={`translate-button-wrapper ${className}`}>
      <button
        onClick={toggleTranslation}
        disabled={isTranslating}
        className={`translate-button ${variantClass} ${activeClass} ${loadingClass}`}
        aria-label={isEnabled ? 'Show original English text' : 'Translate page to Urdu'}
        title={isEnabled ? 'Switch back to English' : 'ترجمہ - Translate to Urdu'}
        type="button"
      >
        <span className="translate-button__content">
          {isTranslating ? (
            <LoadingSpinner size={16} />
          ) : (
            <TranslateIcon size={18} active={isEnabled} />
          )}

          {showLabel && (
            <span className="translate-button__label">
              {isTranslating ? 'Translating...' : buttonLabel}
            </span>
          )}
        </span>
      </button>

      {/* Error tooltip */}
      {error && (
        <div className="translate-button__error" role="alert">
          <span className="translate-button__error-icon">⚠️</span>
          <span className="translate-button__error-message">{error}</span>
        </div>
      )}
    </div>
  );
};

export default NavbarTranslateButton;
