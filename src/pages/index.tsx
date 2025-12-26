import React, { useState, useEffect } from 'react';
import Link from '@docusaurus/Link';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import LanguageToggle from '../components/Language/LanguageToggle';
import { LanguageProvider } from '../components/Language/LanguageContext';

// Translations
const translations = {
  en: {
    title: "Ary's Physical & Humanoid Robotics",
    subtitle: "A comprehensive guide to the fascinating world of Physical AI and humanoid robotics, from fundamental concepts to advanced applications in embodied intelligence.",
    startLearning: "Start Learning",
    viewChapters: "View Chapters",
    featuresTitle: "Why Choose This Robotics Book?",
    featuresSubtitle: "Experience a revolutionary approach to learning Physical AI and humanoid robotics",
    chaptersTitle: "13-Week Complete Course",
    chaptersSubtitle: "Master Physical AI and humanoid robotics from foundations to advanced applications",
    features: [
      {
        icon: '🧠',
        title: 'Physical AI & Embodied Intelligence',
        description: 'Learn how AI systems interact with the physical world through sensors, actuators, and real-time decision making.'
      },
      {
        icon: '🤖',
        title: 'ROS 2 Robot Operating System',
        description: 'Master the industry-standard framework for building robot applications with modern tools and best practices.'
      },
      {
        icon: '🎮',
        title: 'Robot Simulation',
        description: 'Develop and test algorithms in Gazebo and NVIDIA Isaac Sim before deploying to real hardware.'
      },
      {
        icon: '🦿',
        title: 'Humanoid Development',
        description: 'Explore bipedal locomotion, manipulation, and human-robot interaction with Unitree and other platforms.'
      },
      {
        icon: '🗣️',
        title: 'Conversational AI',
        description: 'Integrate GPT models and speech systems for natural human-robot communication.'
      },
      {
        icon: '🔧',
        title: 'Hands-On Examples',
        description: 'Practical code examples with ROS 2, Python, and real hardware integration patterns.'
      }
    ],
    chapters: [
      {
        icon: '🧠',
        title: 'Physical AI Foundations',
        description: 'Introduction to embodied intelligence, sensors, and the transition from digital AI to physical systems.',
        link: '/docs/part-1-foundations/introduction-to-physical-ai',
        level: 'Week 1-2',
        duration: '20 hours'
      },
      {
        icon: '🔧',
        title: 'ROS 2 Fundamentals',
        description: 'Master robot operating system concepts including nodes, topics, services, and actions.',
        link: '/docs/part-2-ros2/ros2-fundamentals',
        level: 'Week 3-5',
        duration: '40 hours'
      },
      {
        icon: '🎮',
        title: 'Robot Simulation',
        description: 'Build and test robots in Gazebo and Unity with realistic physics and sensor simulation.',
        link: '/docs/part-3-simulation/gazebo-unity-simulation',
        level: 'Week 6-7',
        duration: '35 hours'
      },
      {
        icon: '🎯',
        title: 'NVIDIA Isaac Platform',
        description: 'GPU-accelerated simulation, AI perception, and reinforcement learning for robots.',
        link: '/docs/part-4-isaac/nvidia-isaac-platform',
        level: 'Week 8-10',
        duration: '45 hours'
      },
      {
        icon: '🦿',
        title: 'Humanoid Development',
        description: 'Kinematics, bipedal locomotion, manipulation, and natural human-robot interaction.',
        link: '/docs/part-5-humanoid/humanoid-robot-development',
        level: 'Week 11-12',
        duration: '40 hours'
      },
      {
        icon: '🗣️',
        title: 'Conversational Robotics',
        description: 'Integrate GPT models, speech recognition, and multi-modal interaction for natural dialogue.',
        link: '/docs/part-6-conversational/conversational-robotics',
        level: 'Week 13',
        duration: '20 hours'
      }
    ]
  },
  ur: {
    title: "آریہ کی فزیکل اور ہیومینائڈ روبوٹکس",
    subtitle: "فزیکل AI اور ہیومینائڈ روبوٹکس کی دلچسپ دنیا کے لیے ایک جامع گائیڈ، بنیادی تصورات سے لے کر مجسم ذہانت میں جدید ایپلی کیشنز تک۔",
    startLearning: "سیکھنا شروع کریں",
    viewChapters: "ابواب دیکھیں",
    featuresTitle: "اس روبوٹکس کتاب کو کیوں منتخب کریں؟",
    featuresSubtitle: "فزیکل AI اور ہیومینائڈ روبوٹکس سیکھنے کے لیے ایک انقلابی طریقہ کا تجربہ کریں",
    chaptersTitle: "13 ہفتوں کا مکمل کورس",
    chaptersSubtitle: "بنیادی سے لے کر جدید ایپلی کیشنز تک فزیکل AI اور ہیومینائڈ روبوٹکس میں مہارت حاصل کریں",
    features: [
      {
        icon: '🧠',
        title: 'فزیکل AI اور مجسم ذہانت',
        description: 'سینسرز، ایکچویٹرز اور حقیقی وقت کے فیصلہ سازی کے ذریعے AI سسٹم کس طرح جسمانی دنیا کے ساتھ تعامل کرتے ہیں سیکھیں۔'
      },
      {
        icon: '🤖',
        title: 'ROS 2 روبوٹ آپریٹنگ سسٹم',
        description: 'جدید ٹولز اور بہترین طریقوں کے ساتھ روبوٹ ایپلیکیشنز بنانے کے لیے صنعت کے معیاری فریم ورک میں مہارت حاصل کریں۔'
      },
      {
        icon: '🎮',
        title: 'روبوٹ سمیولیشن',
        description: 'حقیقی ہارڈویئر پر تعینات کرنے سے پہلے Gazebo اور NVIDIA Isaac Sim میں الگورتھم تیار اور جانچیں۔'
      },
      {
        icon: '🦿',
        title: 'ہیومینائڈ ڈیولپمنٹ',
        description: 'Unitree اور دیگر پلیٹ فارمز کے ساتھ دو پیروں پر چلنا، ہیرا پھیری، اور انسان-روبوٹ تعامل کو دریافت کریں۔'
      },
      {
        icon: '🗣️',
        title: 'گفتگو کی AI',
        description: 'قدرتی انسان-روبوٹ مواصلات کے لیے GPT ماڈلز اور تقریر سسٹم کو مربوط کریں۔'
      },
      {
        icon: '🔧',
        title: 'عملی مثالیں',
        description: 'ROS 2، Python، اور حقیقی ہارڈویئر انضمام کے نمونوں کے ساتھ عملی کوڈ کی مثالیں۔'
      }
    ],
    chapters: [
      {
        icon: '🧠',
        title: 'فزیکل AI کی بنیادیں',
        description: 'مجسم ذہانت، سینسرز، اور ڈیجیٹل AI سے جسمانی نظام میں منتقلی کا تعارف۔',
        link: '/docs/part-1-foundations/introduction-to-physical-ai',
        level: 'ہفتہ 1-2',
        duration: '20 گھنٹے'
      },
      {
        icon: '🔧',
        title: 'ROS 2 بنیادیں',
        description: 'نوڈز، ٹاپکس، سروسز، اور ایکشنز سمیت روبوٹ آپریٹنگ سسٹم کے تصورات میں مہارت حاصل کریں۔',
        link: '/docs/part-2-ros2/ros2-fundamentals',
        level: 'ہفتہ 3-5',
        duration: '40 گھنٹے'
      },
      {
        icon: '🎮',
        title: 'روبوٹ سمیولیشن',
        description: 'حقیقت پسندانہ طبیعیات اور سینسر سمیولیشن کے ساتھ Gazebo اور Unity میں روبوٹ بنائیں اور جانچیں۔',
        link: '/docs/part-3-simulation/gazebo-unity-simulation',
        level: 'ہفتہ 6-7',
        duration: '35 گھنٹے'
      },
      {
        icon: '🎯',
        title: 'NVIDIA Isaac پلیٹ فارم',
        description: 'روبوٹس کے لیے GPU-تیز شدہ سمیولیشن، AI ادراک، اور تقویتی سیکھنا۔',
        link: '/docs/part-4-isaac/nvidia-isaac-platform',
        level: 'ہفتہ 8-10',
        duration: '45 گھنٹے'
      },
      {
        icon: '🦿',
        title: 'ہیومینائڈ ڈیولپمنٹ',
        description: 'کائنیمیٹکس، دو پیروں پر چلنا، ہیرا پھیری، اور قدرتی انسان-روبوٹ تعامل۔',
        link: '/docs/part-5-humanoid/humanoid-robot-development',
        level: 'ہفتہ 11-12',
        duration: '40 گھنٹے'
      },
      {
        icon: '🗣️',
        title: 'گفتگو کی روبوٹکس',
        description: 'قدرتی مکالمے کے لیے GPT ماڈلز، تقریر کی شناخت، اور کثیر موڈل تعامل کو مربوط کریں۔',
        link: '/docs/part-6-conversational/conversational-robotics',
        level: 'ہفتہ 13',
        duration: '20 گھنٹے'
      }
    ]
  }
};

// Floating Controls Component (Language + Theme)
function FloatingControls() {
  return (
    <div className="floating-controls">
      <LanguageToggle />
      <ThemeToggle />
    </div>
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
  const {i18n} = useDocusaurusContext();
  const locale = i18n.currentLocale;
  const t = translations[locale] || translations.en;

  return (
    <section className="hero">
      <div className="container">
        <div className="hero-content animate-fade-in-up">
          <h1 className="hero-title">
            {t.title}
          </h1>
          <p className="hero-subtitle">
            {t.subtitle}
          </p>
          <div className="hero-actions">
            <Link
              to="/docs"
              className="btn btn-primary"
            >
              <span>{t.startLearning}</span>
            </Link>
            <Link
              to="#chapters"
              className="btn btn-secondary"
            >
              <span>{t.viewChapters}</span>
            </Link>
          </div>
        </div>
      </div>
    </section>
  );
}

// Features Section
function Features() {
  const {i18n} = useDocusaurusContext();
  const locale = i18n.currentLocale;
  const t = translations[locale] || translations.en;

  return (
    <section id="features" className="features">
      <div className="container">
        <div className="features-header">
          <h2 className="features-title">{t.featuresTitle}</h2>
          <p className="features-subtitle">
            {t.featuresSubtitle}
          </p>
        </div>
        <div className="features-grid">
          {t.features.map((feature, index) => (
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
  const {i18n} = useDocusaurusContext();
  const locale = i18n.currentLocale;
  const t = translations[locale] || translations.en;

  return (
    <section id="chapters" className="chapters">
      <div className="container">
        <div className="chapters-header">
          <h2 className="chapters-title">{t.chaptersTitle}</h2>
          <p className="chapters-subtitle">
            {t.chaptersSubtitle}
          </p>
        </div>
        <div className="chapters-grid">
          {t.chapters.map((chapter, index) => (
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
    <LanguageProvider>
      <Layout
        title="Ary's Physical & Humanoid Robotics"
        description="A comprehensive guide to Physical AI and Humanoid Robotics">
        <a href="#main-content" className="skip-to-main">
          Skip to main content
        </a>
        <main id="main-content">
          <Hero />
          <Features />
          <Chapters />
        </main>
        <SocialLinks />
        <FloatingControls />
      </Layout>
    </LanguageProvider>
  );
}
