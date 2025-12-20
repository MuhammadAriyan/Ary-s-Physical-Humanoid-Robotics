import React, { useState, useEffect } from 'react';
import Link from '@docusaurus/Link';
import Layout from '@theme/Layout';

// Navigation Component
function Navbar() {
  const [isScrolled, setIsScrolled] = useState(false);

  useEffect(() => {
    const handleScroll = () => {
      setIsScrolled(window.scrollY > 10);
    };
    window.addEventListener('scroll', handleScroll);
    return () => window.removeEventListener('scroll', handleScroll);
  }, []);

  return (
    <nav className={`navbar ${isScrolled ? 'scrolled' : ''}`}>
      <div className="navbar-container">
        <Link to="/" className="navbar-logo">
          Ary's Robotics
        </Link>
        <div className="navbar-menu">
        <Link to="/docs/humanoid-robotics-course/introduction-to-humanoid-robotics" className="navbar-link">
          Book
        </Link>
          <Link to="#features" className="navbar-link">
            Features
          </Link>
          <ThemeToggle />
        </div>
      </div>
    </nav>
  );
}

// Theme Toggle Component
function ThemeToggle() {
  const [isDark, setIsDark] = useState(false);

  useEffect(() => {
    const savedTheme = localStorage.getItem('theme');
    const prefersDark = window.matchMedia('(prefers-color-scheme: dark)').matches;
    const theme = savedTheme || (prefersDark ? 'dark' : 'light');
    
    document.documentElement.setAttribute('data-theme', theme);
    setIsDark(theme === 'dark');
  }, []);

  const toggleTheme = () => {
    const newTheme = isDark ? 'light' : 'dark';
    document.documentElement.setAttribute('data-theme', newTheme);
    localStorage.setItem('theme', newTheme);
    setIsDark(!isDark);
  };

  return (
    <button 
      className="theme-toggle" 
      onClick={toggleTheme}
      aria-label="Toggle theme"
    >
      <div className="theme-toggle-slider">
        {isDark ? '🌙' : '☀️'}
      </div>
    </button>
  );
}

// Hero Section
function Hero() {
  return (
    <section className="hero">
      <div className="container">
        <div className="hero-content animate-fade-in-up">
          <h1 className="hero-title">
            Ary's Physical & Humanoid Robotics
          </h1>
          <p className="hero-subtitle">
            A comprehensive guide to fascinating world of robotics, 
            from fundamental concepts to advanced applications in humanoid systems.
          </p>
          <div className="hero-actions">
            <Link
              to="/docs/humanoid-robotics-course/introduction-to-humanoid-robotics"
              className="btn btn-primary"
            >
              <span>Start Learning</span>
            </Link>
            <Link
              to="#chapters"
              className="btn btn-secondary"
            >
              <span>View Chapters</span>
            </Link>
          </div>
        </div>
      </div>
    </section>
  );
}

// Features Section
function Features() {
  const features = [
    {
      icon: '🧠',
      title: 'AI-Driven Content',
      description: 'Continuously updated with the latest advancements in robotics research and technology.'
    },
    {
      icon: '📚',
      title: 'Comprehensive Coverage',
      description: 'From basic principles to advanced humanoid systems, covering all essential topics.'
    },
    {
      icon: '🔧',
      title: 'Practical Examples',
      description: 'Real-world applications and hands-on examples to reinforce learning.'
    },
    {
      icon: '✨',
      title: 'Interactive Learning',
      description: 'Engaging content with visualizations and interactive demonstrations.'
    },
    {
      icon: '🎯',
      title: 'Progressive Path',
      description: 'Carefully structured curriculum from beginner to advanced topics.'
    },
    {
      icon: '🌟',
      title: 'Always Current',
      description: 'Content evolves with the field of robotics and latest best practices.'
    }
  ];

  return (
    <section id="features" className="features">
      <div className="container">
        <div className="features-header">
          <h2 className="features-title">Why Choose This Robotics Book?</h2>
          <p className="features-subtitle">
            Experience a revolutionary approach to learning robotics with cutting-edge content
          </p>
        </div>
        <div className="features-grid">
          {features.map((feature, index) => (
            <div key={index} className={`feature-card animate-fade-in-up animate-stagger-${index + 1}`}>
              <div className="feature-icon">{feature.icon}</div>
              <h3 className="feature-title">{feature.title}</h3>
              <p className="feature-description">{feature.description}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

// Chapters Section
function Chapters() {
  const chapters = [
    {
      icon: '🤖',
      title: 'Introduction to Humanoid Robotics',
      description: 'Begin your journey into the fascinating world of humanoid robots and their fundamental concepts.',
      link: '/docs/humanoid-robotics-course/introduction-to-humanoid-robotics',
      level: 'Beginner',
      duration: '15 min'
    },
    {
      icon: '👁️',
      title: 'Sensors and Perception',
      description: 'Explore how robots perceive and understand their environment through advanced sensing technologies.',
      link: '/docs/humanoid-robotics-course/sensors-and-perception',
      level: 'Beginner',
      duration: '20 min'
    },
    {
      icon: '⚙️',
      title: 'Actuators and Movement',
      description: 'Learn about mechanical systems that enable robots to move and interact with the physical world.',
      link: '/docs/humanoid-robotics-course/actuators-and-movement',
      level: 'Intermediate',
      duration: '25 min'
    },
    {
      icon: '🎮',
      title: 'Control Systems',
      description: 'Master algorithms and systems that govern robot behavior and decision-making processes.',
      link: '/docs/humanoid-robotics-course/control-systems',
      level: 'Intermediate',
      duration: '30 min'
    },
    {
      icon: '🗺️',
      title: 'Path Planning and Navigation',
      description: 'Understand how robots navigate complex environments and plan optimal paths to their goals.',
      link: '/docs/humanoid-robotics-course/path-planning-and-navigation',
      level: 'Advanced',
      duration: '35 min'
    },
    {
      icon: '🚀',
      title: 'Advanced Topics',
      description: 'Explore cutting-edge research in machine learning, computer vision, and human-robot interaction.',
      link: '/docs/humanoid-robotics-course/introduction-to-humanoid-robotics',
      level: 'Advanced',
      duration: '40 min'
    }
  ];

  return (
    <section id="chapters" className="chapters">
      <div className="container">
        <div className="chapters-header">
          <h2 className="chapters-title">Complete Course Curriculum</h2>
          <p className="chapters-subtitle">
            Master humanoid robotics from fundamentals to advanced applications
          </p>
        </div>
        <div className="chapters-grid">
          {chapters.map((chapter, index) => (
            <Link key={index} to={chapter.link} className={`chapter-card animate-fade-in-up animate-stagger-${index + 1}`}>
              <div className="chapter-header">
                <div className="chapter-icon">{chapter.icon}</div>
                <h3 className="chapter-title">{chapter.title}</h3>
              </div>
              <p className="chapter-description">{chapter.description}</p>
              <div className="chapter-meta">
                <span className="chapter-tag">{chapter.level}</span>
                <span className="chapter-tag">{chapter.duration}</span>
              </div>
            </Link>
          ))}
        </div>
      </div>
    </section>
  );
}

// Social Links Component
function SocialLinks() {
  return (
    <div className="social-links">
      <a
        href="https://linkedin.com/in/muhammad-aryan"
        target="_blank"
        rel="noopener noreferrer"
        className="social-link"
        aria-label="LinkedIn"
      >
        <svg viewBox="0 0 24 24" fill="currentColor" width="24" height="24">
          <path d="M20.447 20.452h-3.554v-5.569c0-1.328-.027-3.037-1.852-3.037-1.853 0-2.136 1.445-2.136 2.939v5.667H9.351V9h3.414v1.561h.046c.477-.9 1.637-1.85 3.37-1.85 3.601 0 4.267 2.37 4.267 5.455v6.286zM5.337 7.433c-1.144 0-2.063-.926-2.063-2.065 0-1.138.92-2.063 2.063-2.063 1.14 0 2.064.925 2.064 2.063 0 1.139-.925 2.065-2.064 2.065zm1.782 13.019H3.555V9h3.564v11.452zM22.225 0H1.771C.792 0 0 .774 0 1.729v20.542C0 23.227.792 24 1.771 24h20.451C23.2 24 24 23.227 24 22.271V1.729C24 .774 23.2 0 22.222 0h.003z"/>
        </svg>
      </a>
      <a
        href="https://github.com/MuhammadAriyan/Ary-s-Physical-Humanoid-Robotics"
        target="_blank"
        rel="noopener noreferrer"
        className="social-link"
        aria-label="GitHub"
      >
        <svg viewBox="0 0 24 24" fill="currentColor" width="24" height="24">
          <path d="M12 0c-6.626 0-12 5.373-12 12 0 5.302 3.438 9.8 8.207 11.387.599.111.793-.261.793-.577v-2.234c-3.338.726-4.033-1.416-4.033-1.416-.546-1.387-1.333-1.756-1.333-1.756-1.089-.745.083-.729.083-.729 1.205.084 1.839 1.237 1.839 1.237 1.07 1.834 2.807 1.304 3.492.997.107-.775.418-1.305.762-1.604-2.665-.305-5.467-1.334-5.467-5.931 0-1.311.469-2.381 1.236-3.221-.124-.303-.535-1.524.117-3.176 0 0 1.008-.322 3.301 1.23.957-.266 1.983-.399 3.003-.404 1.02.005 2.047.138 3.006.404 2.291-1.552 3.297-1.23 3.297-1.23.653 1.653.242 2.874.118 3.176.77.84 1.235 1.911 1.235 3.221 0 4.609-2.807 5.624-5.479 5.921.43.372.823 1.102.823 2.222v3.293c0 .319.192.694.801.576 4.765-1.589 8.199-6.086 8.199-11.386 0-6.627-5.373-12-12-12z"/>
        </svg>
      </a>
    </div>
  );
}

// Main Component
export default function Home(): React.ReactNode {
  return (
    <Layout
      title="Ary's Physical & Humanoid Robotics"
      description="A comprehensive guide to Physical & Humanoid Robotics">
      <a href="#main-content" className="skip-to-main">
        Skip to main content
      </a>
      <Navbar />
      <main id="main-content">
        <Hero />
        <Features />
        <Chapters />
      </main>
      <SocialLinks />
    </Layout>
  );
}