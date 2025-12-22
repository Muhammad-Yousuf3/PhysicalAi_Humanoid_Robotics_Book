/**
 * AuthButtons component for the navbar.
 * Shows Sign In / Sign Up buttons or user menu when authenticated.
 */

import { useAuthContext } from '../../hooks/useAuth';
import styles from './styles.module.css';

export default function AuthButtons() {
  const { user, isAuthenticated, signOut, openAuthModal } = useAuthContext();

  if (isAuthenticated && user) {
    const initials = user.name
      ? user.name.split(' ').map(n => n[0]).join('').toUpperCase().slice(0, 2)
      : user.email[0].toUpperCase();

    return (
      <div className={styles.authButtons}>
        <div className={styles.userMenu}>
          <div className={styles.userInfo}>
            <span className={styles.userAvatar}>{initials}</span>
            <span className={styles.userName}>{user.name || user.email}</span>
          </div>
          <button className={styles.signOutButton} onClick={signOut}>
            Sign Out
          </button>
        </div>
      </div>
    );
  }

  return (
    <div className={styles.authButtons}>
      <button
        className={styles.signInButton}
        onClick={() => openAuthModal('signin')}
      >
        Sign In
      </button>
      <button
        className={styles.signUpButton}
        onClick={() => openAuthModal('signup')}
      >
        Sign Up
      </button>
    </div>
  );
}
