import React from 'react';
import Header from '@site/src/components/Header';

/**
 * Custom Navbar wrapper that replaces the default Docusaurus navbar
 * with our custom Header component featuring glassmorphism design
 */
export default function NavbarWrapper(props) {
  return <Header />;
}
