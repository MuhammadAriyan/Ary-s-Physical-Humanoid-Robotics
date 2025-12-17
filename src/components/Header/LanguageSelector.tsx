import React, { useState, useCallback, useEffect, useRef } from 'react';
import { ChevronIcon } from './icons';
import styles from './LanguageSelector.module.css';

/**
 * Locale configuration interface
 */
export interface Locale {
  /** Language code (e.g., 'en', 'es') */
  code: string;
  /** Display label (e.g., 'English', 'Espanol') */
  label: string;
  /** Flag emoji for visual representation */
  flag: string;
}

/**
 * Available locales - i18n ready for future expansion
 */
const LOCALES: Locale[] = [
  { code: 'en', label: 'English', flag: '\u{1F1FA}\u{1F1F8}' },
  // Future locales can be added here:
  // { code: 'es', label: 'Espanol', flag: '\u{1F1EA}\u{1F1F8}' },
  // { code: 'fr', label: 'Francais', flag: '\u{1F1EB}\u{1F1F7}' },
  // { code: 'de', label: 'Deutsch', flag: '\u{1F1E9}\u{1F1EA}' },
  // { code: 'ja', label: 'Japanese', flag: '\u{1F1EF}\u{1F1F5}' },
];

export interface LanguageSelectorProps {
  /** Additional CSS class name */
  className?: string;
  /** Whether to display in compact mode (flag only) */
  compact?: boolean;
}

/**
 * LanguageSelector component - dropdown for language/locale selection
 * Currently supports English only, but is i18n ready for future locales
 */
export function LanguageSelector({
  className,
  compact = false,
}: LanguageSelectorProps): React.ReactElement {
  const [isOpen, setIsOpen] = useState(false);
  const [currentLocale, setCurrentLocale] = useState<Locale>(LOCALES[0]);
  const containerRef = useRef<HTMLDivElement>(null);
  const buttonRef = useRef<HTMLButtonElement>(null);

  // Handle click outside to close dropdown
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (
        containerRef.current &&
        !containerRef.current.contains(event.target as Node)
      ) {
        setIsOpen(false);
      }
    };

    if (isOpen) {
      document.addEventListener('mousedown', handleClickOutside);
    }

    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [isOpen]);

  // Handle escape key to close dropdown
  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      if (event.key === 'Escape' && isOpen) {
        setIsOpen(false);
        buttonRef.current?.focus();
      }
    };

    document.addEventListener('keydown', handleKeyDown);
    return () => {
      document.removeEventListener('keydown', handleKeyDown);
    };
  }, [isOpen]);

  const toggleDropdown = useCallback(() => {
    setIsOpen((prev) => !prev);
  }, []);

  const handleLocaleSelect = useCallback((locale: Locale) => {
    setCurrentLocale(locale);
    setIsOpen(false);
    buttonRef.current?.focus();
    // Future: integrate with Docusaurus i18n or other i18n library
    // e.g., router.push(router.asPath, router.asPath, { locale: locale.code });
  }, []);

  const handleKeyboardSelect = useCallback(
    (event: React.KeyboardEvent, locale: Locale) => {
      if (event.key === 'Enter' || event.key === ' ') {
        event.preventDefault();
        handleLocaleSelect(locale);
      }
    },
    [handleLocaleSelect]
  );

  const containerClassName = className
    ? `${styles.languageSelector} ${className}`
    : styles.languageSelector;

  // Don't render dropdown if only one locale available
  const hasMultipleLocales = LOCALES.length > 1;

  return (
    <div ref={containerRef} className={containerClassName}>
      <button
        ref={buttonRef}
        className={styles.selectorButton}
        onClick={toggleDropdown}
        aria-expanded={isOpen}
        aria-haspopup="listbox"
        aria-label={`Select language. Current: ${currentLocale.label}`}
        type="button"
        disabled={!hasMultipleLocales}
      >
        <span className={styles.flag} aria-hidden="true">
          {currentLocale.flag}
        </span>
        {!compact && (
          <span className={styles.label}>{currentLocale.label}</span>
        )}
        {hasMultipleLocales && (
          <ChevronIcon
            className={styles.chevron}
            size={14}
            direction={isOpen ? 'up' : 'down'}
          />
        )}
      </button>

      {isOpen && hasMultipleLocales && (
        <ul
          className={styles.dropdown}
          role="listbox"
          aria-label="Available languages"
        >
          {LOCALES.map((locale) => (
            <li
              key={locale.code}
              role="option"
              aria-selected={locale.code === currentLocale.code}
              className={`${styles.dropdownItem} ${
                locale.code === currentLocale.code ? styles.active : ''
              }`}
              onClick={() => handleLocaleSelect(locale)}
              onKeyDown={(e) => handleKeyboardSelect(e, locale)}
              tabIndex={0}
            >
              <span className={styles.flag} aria-hidden="true">
                {locale.flag}
              </span>
              <span className={styles.label}>{locale.label}</span>
              {locale.code === currentLocale.code && (
                <span className={styles.checkmark} aria-hidden="true">
                  &#10003;
                </span>
              )}
            </li>
          ))}
        </ul>
      )}
    </div>
  );
}

export default LanguageSelector;
