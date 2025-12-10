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
    </Layout>
  );
}