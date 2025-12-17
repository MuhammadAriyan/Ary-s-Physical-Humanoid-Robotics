import React, { useState, useEffect, useRef } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { useLocation, useHistory } from '@docusaurus/router';
import styles from './styles.module.css';

interface LocaleConfig {
  locale: string;
  label: string;
  direction: string;
  htmlLang: string;
}

interface LanguageOption {
  locale: string;
  label: string;
  icon: string;
}

const LanguageToggle: React.FC = () => {
  const { i18n } = useDocusaurusContext();
  const location = useLocation();
  const history = useHistory();
  const [isOpen, setIsOpen] = useState(false);
  const [dropdownPosition, setDropdownPosition] = useState({ top: 'calc(100% + 8px)', left: '0', right: 'auto' });
  const dropdownRef = useRef<HTMLDivElement>(null);
  const dropdownMenuRef = useRef<HTMLDivElement>(null);

  // Get current locale information
  const currentLocale = i18n.currentLocale;
  const localeConfigs = i18n.locales.reduce((configs: Record<string, LocaleConfig>, locale: string) => {
    configs[locale] = {
      locale,
      label: i18n.localeConfigs[locale]?.label || locale,
      direction: i18n.localeConfigs[locale]?.direction || 'ltr',
      htmlLang: i18n.localeConfigs[locale]?.htmlLang || locale,
    };
    return configs;
  }, {});

  // Toggle dropdown visibility
  const toggleDropdown = () => {
    setIsOpen(!isOpen);
  };

  // Handle language selection
  const selectLanguage = (locale: string) => {
    // Build the path for the selected locale
    const pathWithoutLocale = location.pathname.replace(/^\/(en|ur)/, '') || '/';
    const newPath = locale === 'en' ? pathWithoutLocale : `/${locale}${pathWithoutLocale}`;

    // Save preference to localStorage
    try {
      localStorage.setItem('preferredLocale', locale);
    } catch (e) {
      // localStorage may be unavailable or disabled
      console.warn('Unable to save locale preference:', e);
    }

    // Update the URL to switch locale
    history.push(newPath);
    setIsOpen(false);
  };

  // Handle dropdown positioning to prevent going off-screen
  useEffect(() => {
    if (isOpen && dropdownRef.current) {
      const buttonRect = dropdownRef.current.getBoundingClientRect();
      const viewportWidth = window.innerWidth;
      const dropdownWidth = 150; // Approximate width of dropdown (120px min + padding)

      // Calculate available space on the right
      const spaceToRight = viewportWidth - buttonRect.right;

      // If dropdown would go off-screen on the right, position it to the left
      if (spaceToRight < dropdownWidth) {
        setDropdownPosition({
          top: 'calc(100% + 8px)',
          left: 'auto',
          right: '0'
        });
      } else {
        // Otherwise, position it normally to the left
        setDropdownPosition({
          top: 'calc(100% + 8px)',
          left: '0',
          right: 'auto'
        });
      }
    }
  }, [isOpen]);

  // Close dropdown when clicking outside or pressing Escape
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (dropdownRef.current && !dropdownRef.current.contains(event.target as Node)) {
        setIsOpen(false);
      }
    };

    const handleKeyDown = (event: KeyboardEvent) => {
      if (event.key === 'Escape') {
        setIsOpen(false);
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    document.addEventListener('keydown', handleKeyDown);
    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
      document.removeEventListener('keydown', handleKeyDown);
    };
  }, []);

  return (
    <div className={styles.languageToggleContainer} ref={dropdownRef}>
      <button
        className={styles.languageToggle}
        onClick={toggleDropdown}
        aria-expanded={isOpen}
        aria-haspopup="listbox"
        aria-label={`Language selector, currently ${localeConfigs[currentLocale]?.label || currentLocale}`}
        title={`Current language: ${localeConfigs[currentLocale]?.label || currentLocale}`}
      >
        <span className={styles.languageToggleIcon}>🌐</span>
        <span className={styles.languageCode}>{currentLocale.toUpperCase()}</span>
      </button>

      {isOpen && (
        <div
          className={styles.languageToggleDropdown}
          style={dropdownPosition}
          role="listbox"
          aria-label="Language options"
        >
          {i18n.locales.map((locale: string) => {
            const isCurrentLocale = locale === currentLocale;
            return (
              <button
                key={locale}
                className={`${styles.languageToggleOption} ${isCurrentLocale ? styles.languageToggleOptionActive : ''}`}
                onClick={() => selectLanguage(locale)}
                aria-label={`Switch to ${localeConfigs[locale].label}${isCurrentLocale ? ' (current)' : ''}`}
                aria-selected={isCurrentLocale}
                role="option"
              >
                {localeConfigs[locale].label}
                {isCurrentLocale && <span className={styles.checkmark}>✓</span>}
              </button>
            );
          })}
        </div>
      )}
    </div>
  );
};

export default LanguageToggle;