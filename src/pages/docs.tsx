import React, { useState, useEffect } from 'react';
import Link from '@docusaurus/Link';
import clsx from 'clsx';
import styles from '../theme/FubuniChatInjector/style.module.css';

export default function DocsLanding(): React.ReactNode {
  // Start with 'intro' - chapters will only appear after intro animation completes
  const [phase, setPhase] = useState<'intro' | 'title' | 'arrow' | 'exit' | 'chapters'>('intro');
  const [reducedMotion, setReducedMotion] = useState(false);

  useEffect(() => {
    // Check for reduced motion preference
    const prefersReducedMotion = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
    setReducedMotion(prefersReducedMotion);

    if (prefersReducedMotion) {
      setPhase('chapters');
      return;
    }

    // Animation sequence: intro → title → arrow → exit → chapters
    const timers = [
      setTimeout(() => setPhase('title'), 800),
      setTimeout(() => setPhase('arrow'), 1600),
      setTimeout(() => setPhase('exit'), 2500),
      setTimeout(() => setPhase('chapters'), 3200),
    ];

    return () => timers.forEach(clearTimeout);
  }, []);

  return (
    <div className={styles.container}>
      {/* Home Icon */}
      <Link to="/" className={styles.homeIcon} aria-label="Go to home">
        <svg viewBox="0 0 24 24" fill="currentColor" stroke="none">
          <path d="M3 9l9-7 9 7v11a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2z" />
        </svg>
      </Link>

      {/* Intro Section */}
      <div
        className={clsx(
          styles.intro,
          phase !== 'chapters' && styles.introVisible,
          phase === 'exit' && styles.introExit,
          phase === 'chapters' && styles.introHidden
        )}
      >
        <h1 className={styles.journeyText}>Journey into</h1>
        <h2 className={styles.titleText}>Ary's Humanoid Robots and Physical AI</h2>
        <div
          className={clsx(
            styles.arrowContainer,
            (phase === 'arrow' || phase === 'exit') && styles.arrowVisible
          )}
        >
          <svg
            className={clsx(styles.arrow, phase === 'exit' && styles.arrowExit)}
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
          >
            <path d="M7 17L17 7M17 7V7H7M17 7V17" />
          </svg>
        </div>
      </div>

      {/* Chapters Navigation */}
      <div
        className={clsx(
          styles.chapters,
          phase === 'chapters' && styles.chaptersVisible
        )}
      >
        <div className={styles.chaptersHeader}>
          <h2 className={styles.chaptersTitle}>13-Week Complete Course</h2>
          <p className={styles.chaptersSubtitle}>
            Master Physical AI and humanoid robotics from foundations to advanced applications
          </p>
        </div>

        <div className={styles.chaptersGrid}>
          {/* Part 1 */}
          <Link to="/docs/part-1-foundations/introduction-to-physical-ai" className={styles.chapterCard}>
            <div className={styles.chapterIcon}>🧠</div>
            <h3 className={styles.chapterTitle}>Physical AI Foundations</h3>
            <p className={styles.chapterDesc}>Introduction to embodied intelligence, sensors, and the transition from digital AI to physical systems.</p>
            <div className={styles.chapterMeta}>
              <span className={styles.chapterTag}>Week 1-2</span>
              <span className={styles.chapterDuration}>20 hours</span>
            </div>
          </Link>

          {/* Part 2 */}
          <Link to="/docs/part-2-ros2/ros2-fundamentals" className={styles.chapterCard}>
            <div className={styles.chapterIcon}>🔧</div>
            <h3 className={styles.chapterTitle}>ROS 2 Fundamentals</h3>
            <p className={styles.chapterDesc}>Master robot operating system concepts including nodes, topics, services, and actions.</p>
            <div className={styles.chapterMeta}>
              <span className={styles.chapterTag}>Week 3-5</span>
              <span className={styles.chapterDuration}>40 hours</span>
            </div>
          </Link>

          {/* Part 3 */}
          <Link to="/docs/part-3-simulation/gazebo-unity-simulation" className={styles.chapterCard}>
            <div className={styles.chapterIcon}>🎮</div>
            <h3 className={styles.chapterTitle}>Robot Simulation</h3>
            <p className={styles.chapterDesc}>Build and test robots in Gazebo and Unity with realistic physics and sensor simulation.</p>
            <div className={styles.chapterMeta}>
              <span className={styles.chapterTag}>Week 6-7</span>
              <span className={styles.chapterDuration}>35 hours</span>
            </div>
          </Link>

          {/* Part 4 */}
          <Link to="/docs/part-4-isaac/nvidia-isaac-platform" className={styles.chapterCard}>
            <div className={styles.chapterIcon}>🎯</div>
            <h3 className={styles.chapterTitle}>NVIDIA Isaac Platform</h3>
            <p className={styles.chapterDesc}>GPU-accelerated simulation, AI perception, and reinforcement learning for robots.</p>
            <div className={styles.chapterMeta}>
              <span className={styles.chapterTag}>Week 8-10</span>
              <span className={styles.chapterDuration}>45 hours</span>
            </div>
          </Link>

          {/* Part 5 */}
          <Link to="/docs/part-5-humanoid/humanoid-robot-development" className={styles.chapterCard}>
            <div className={styles.chapterIcon}>🦿</div>
            <h3 className={styles.chapterTitle}>Humanoid Development</h3>
            <p className={styles.chapterDesc}>Kinematics, bipedal locomotion, manipulation, and natural human-robot interaction.</p>
            <div className={styles.chapterMeta}>
              <span className={styles.chapterTag}>Week 11-12</span>
              <span className={styles.chapterDuration}>40 hours</span>
            </div>
          </Link>

          {/* Part 6 */}
          <Link to="/docs/part-6-conversational/conversational-robotics" className={styles.chapterCard}>
            <div className={styles.chapterIcon}>🗣️</div>
            <h3 className={styles.chapterTitle}>Conversational Robotics</h3>
            <p className={styles.chapterDesc}>Integrate GPT models, speech recognition, and multi-modal interaction for natural dialogue.</p>
            <div className={styles.chapterMeta}>
              <span className={styles.chapterTag}>Week 13</span>
              <span className={styles.chapterDuration}>20 hours</span>
            </div>
          </Link>
        </div>

        {/* Appendices */}
        <div className={styles.appendicesSection}>
          <h3 className={styles.appendicesTitle}>Appendices</h3>
          <div className={styles.appendicesLinks}>
            <Link to="/docs/appendix/hardware-specifications" className={styles.appendixLink}>
              A: Hardware Specifications
            </Link>
            <Link to="/docs/appendix/B-simulation-setup" className={styles.appendixLink}>
              B: Simulation Setup Guide
            </Link>
            <Link to="/docs/appendix/C-community-resources" className={styles.appendixLink}>
              C: Community Resources
            </Link>
            <Link to="/docs/appendix/D-assessment-rubrics" className={styles.appendixLink}>
              D: Assessment Rubrics
            </Link>
          </div>
        </div>
      </div>
    </div>
  );
}
