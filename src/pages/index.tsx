import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function WelcomePage() {
  const { siteConfig } = useDocusaurusContext();
  const [contentItems, setContentItems] = useState([]);
  const [searchQuery, setSearchQuery] = useState('');

  useEffect(() => {
    // Dynamic content scanning - all available chapters and sections
    const content = [
      {
        id: 'intro',
        title: 'Introduction to Physical Robotics',
        description: 'Core concepts and foundations',
        path: '/docs/intro',
        category: 'Foundations'
      },
      {
        id: 'humanoid-intro',
        title: 'Introduction to Humanoid Robotics',
        description: 'History and evolution of humanoid robots',
        path: '/docs/humanoid-robotics-course/introduction-to-humanoid-robotics',
        category: 'Humanoid Course'
      },
      {
        id: 'sensors',
        title: 'Sensors and Perception',
        description: 'Robotic sensing systems',
        path: '/docs/humanoid-robotics-course/sensors-and-perception',
        category: 'Humanoid Course'
      },
      {
        id: 'actuators',
        title: 'Actuators and Movement',
        description: 'Robot actuation systems',
        path: '/docs/humanoid-robotics-course/actuators-and-movement',
        category: 'Humanoid Course'
      },
      {
        id: 'control',
        title: 'Control Systems',
        description: 'Robot control theory',
        path: '/docs/humanoid-robotics-course/control-systems',
        category: 'Humanoid Course'
      },
      {
        id: 'planning',
        title: 'Path Planning and Navigation',
        description: 'Robot motion planning',
        path: '/docs/humanoid-robotics-course/path-planning-and-navigation',
        category: 'Humanoid Course'
      },
      {
        id: 'intro-to-robotics',
        title: 'Introduction to Robotics',
        description: 'Complete robotics fundamentals',
        path: '/docs/intro-to-robotics/index',
        category: 'Tutorial Basics'
      },
      {
        id: 'part1-foundations',
        title: 'Part 1: Foundations for Beginners',
        description: 'Basic robotics concepts',
        path: '/docs/part1-foundations-for-beginners/introduction-to-robotics',
        category: 'Beginner Course'
      },
      {
        id: 'part2-university',
        title: 'Part 2: University Level',
        description: 'Advanced robotics topics',
        path: '/docs/part2-university-level/actuators-and-grippers',
        category: 'University Course'
      }
    ];
    
    setContentItems(content);
  }, []);

  const filteredContent = contentItems.filter(item =>
    item.title.toLowerCase().includes(searchQuery.toLowerCase()) ||
    item.description.toLowerCase().includes(searchQuery.toLowerCase()) ||
    item.category.toLowerCase().includes(searchQuery.toLowerCase())
  );

  return (
    <Layout
      title={`${siteConfig.title} - Modern Robotics Education`}
      description="Comprehensive robotics education platform with modern UI and dynamic navigation">
      
      {/* Hero Section with Glassmorphism */}
      <section className={clsx('hero hero--glass', styles.heroGlass)}>
        <div className="container">
          <div className={styles.glassCard}>
            <Heading as="h1" className={styles.heroTitle}>
              Physical & Humanoid Robotics
            </Heading>
            <p className={styles.heroSubtitle}>
              University-level education for the next generation of roboticists
            </p>
            <div className={styles.heroButtons}>
              <Link to="/docs/intro" className={styles.modernButtonPrimary}>
                Get Started
              </Link>
              <Link to="/docs/humanoid-robotics-course/introduction-to-humanoid-robotics" className={styles.modernButtonSecondary}>
                Humanoid Course
              </Link>
            </div>
          </div>
        </div>
      </section>

      {/* Search Section */}
      <section className={styles.searchSection}>
        <div className="container">
          <div className={styles.glassCard}>
            <input
              type="text"
              placeholder="Search all chapters and sections..."
              className={styles.searchInput}
              value={searchQuery}
              onChange={(e) => setSearchQuery(e.target.value)}
            />
          </div>
        </div>
      </section>

      {/* Content Grid - All Chapters and Sections */}
      <section className={styles.contentGridSection}>
        <div className="container">
          <h2 className={styles.sectionTitle}>All Chapters & Sections</h2>
          <div className={styles.contentGrid}>
            {filteredContent.map((item) => (
              <div key={item.id} className={styles.contentCard}>
                <div className={styles.cardCategory}>{item.category}</div>
                <h3 className={styles.cardTitle}>{item.title}</h3>
                <p className={styles.cardDescription}>{item.description}</p>
                <Link to={item.path} className={styles.cardLink}>
                  Explore →
                </Link>
              </div>
            ))}
          </div>
        </div>
      </section>

      {/* Features Section */}
      <section className={styles.featuresSection}>
        <div className="container">
          <div className={styles.glassCard}>
            <h2 className={styles.featuresTitle}>Why Choose This Course?</h2>
            <div className={styles.featuresGrid}>
              <div className={styles.featureItem}>
                <div className={styles.featureIcon}>🎓</div>
                <h3>University Level</h3>
                <p>Academic rigor with mathematical foundations</p>
              </div>
              <div className={styles.featureItem}>
                <div className={styles.featureIcon}>🤖</div>
                <h3>Physical Focus</h3>
                <p>Emphasis on real-world robotics implementation</p>
              </div>
              <div className={styles.featureItem}>
                <div className={styles.featureIcon}>📊</div>
                <h3>Visual Learning</h3>
                <p>Rich diagrams and practical examples</p>
              </div>
              <div className={styles.featureItem}>
                <div className={styles.featureIcon}>🚀</div>
                <h3>2025 State-of-the-Art</h3>
                <p>Latest advancements and industrial deployments</p>
              </div>
            </div>
          </div>
        </div>
      </section>
    </Layout>
  );
}

export default WelcomePage;
