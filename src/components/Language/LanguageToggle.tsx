import React from 'react';
import { useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

export default function LanguageToggle() {
  const {i18n} = useDocusaurusContext();
  const location = useLocation();
  const currentLocale = i18n.currentLocale;

  const handleToggle = () => {
    const newLang = currentLocale === 'en' ? 'ur' : 'en';

    // Navigate to the same page in the new locale
    const currentPath = location.pathname;
    const baseUrl = '/Ary-s-Physical-Humanoid-Robotics/';

    // Remove base URL to get the path
    let path = currentPath.replace(baseUrl, '');

    // Remove any existing locale prefix (ur/)
    path = path.replace(/^ur\//, '');

    // Build new path with correct locale
    let newPath;
    if (newLang === 'ur') {
      newPath = baseUrl + 'ur/' + path;
    } else {
      newPath = baseUrl + path;
    }

    window.location.href = newPath;
  };

  return (
    <button
      onClick={handleToggle}
      className="language-toggle"
      aria-label={currentLocale === 'en' ? 'Switch to Urdu' : 'Switch to English'}
      title={currentLocale === 'en' ? 'Switch to Urdu' : 'Switch to English'}
    >
      <span className="language-toggle-option" data-active={currentLocale === 'en'}>
        EN
      </span>
      <span className="language-toggle-separator">/</span>
      <span className="language-toggle-option" data-active={currentLocale === 'ur'}>
        UR
      </span>
    </button>
  );
}
