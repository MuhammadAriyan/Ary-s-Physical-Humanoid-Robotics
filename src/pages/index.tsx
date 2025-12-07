import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HeroSection() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <section className={styles.heroSection}>
      <div className={styles.heroBackground}>
        <div className={styles.heroGrid}></div>
        <div className={styles.heroGradient}></div>
      </div>
      <div className="container">
        <div className={styles.heroContent}>
          <Heading as="h1" className={styles.heroTitle}>
            <span className={styles.titleLine}>Welcome to Ary's</span>
            <span className={styles.titleLine}>Physical & Humanoid</span>
            <span className={styles.titleLine}>Robotics</span>
          </Heading>
          <p className={styles.heroSubtitle}>
            An AI-driven comprehensive guide to the future of robotics
          </p>
          <div className={styles.heroDescription}>
            <p>
              🚀 <strong>Embark on an Extraordinary Journey!</strong> Welcome to your comprehensive gateway 
              to the fascinating world of physical and humanoid robotics. This innovative, AI-powered book 
              provides cutting-edge insights and continuously updated content in the rapidly evolving field 
              of robotics.
            </p>
            <p>
              📚 <strong>What You'll Discover:</strong> From fundamental robotics principles to advanced 
              humanoid systems, each chapter is carefully crafted to build your knowledge progressively. 
              Whether you're a beginner or an experienced engineer, you'll find valuable insights and 
              practical applications that will accelerate your learning journey.
            </p>
            <p>
              🤖 <strong>Why This Book?</strong> Unlike traditional textbooks, this AI-driven resource 
              evolves with the field, ensuring you always have access to the latest advancements and 
              best practices in robotics engineering.
            </p>
          </div>
          <div className={styles.heroActions}>
            <Link
              className={clsx('button button--primary button--lg', styles.primaryButton)}
              to="/docs/humanoid-robotics-course/introduction-to-humanoid-robotics">
              🎯 Start Your Journey
            </Link>
            <Link
              className={clsx('button button--secondary button--lg', styles.secondaryButton)}
              to="#chapters">
              📖 Explore All Chapters
            </Link>
          </div>
        </div>
      </div>
    </section>
  );
}

function ChapterSection() {
  const chapters = [
    {
      title: "01 - Introduction to Humanoid Robotics",
      description: "🎯 Begin your journey into the fascinating world of humanoid robots. Discover their history, fundamental concepts, and the core principles that make them unique. Learn about the evolution from industrial robots to modern humanoid systems.",
      icon: "🤖",
      link: "/docs/humanoid-robotics-course/introduction-to-humanoid-robotics",
      color: "#6366f1",
      duration: "15 min read",
      level: "Beginner",
      topics: ["History of Robotics", "Humanoid Robot Classification", "Fundamental Components", "Applications & Future Trends"]
    },
    {
      title: "02 - Sensors and Perception",
      description: "👁️ Explore how robots perceive and understand their environment through advanced sensing technologies. Master vision systems, proximity sensors, and the fusion of multiple sensor inputs for comprehensive environmental awareness.",
      icon: "👁️",
      link: "/docs/humanoid-robotics-course/sensors-and-perception",
      color: "#8b5cf6",
      duration: "20 min read",
      level: "Beginner",
      topics: ["Computer Vision", "Depth Sensors", "Sensor Fusion", "Environmental Mapping"]
    },
    {
      title: "03 - Actuators and Movement",
      description: "⚙️ Learn about the mechanical systems that enable robots to move and interact with the physical world. Understand servo motors, hydraulic systems, and the biomechanical principles that inspire humanoid robot design.",
      icon: "⚙️",
      link: "/docs/humanoid-robotics-course/actuators-and-movement",
      color: "#ec4899",
      duration: "25 min read",
      level: "Intermediate",
      topics: ["Servo Motors", "Hydraulic Actuators", "Biomechanics", "Joint Design"]
    },
    {
      title: "04 - Control Systems",
      description: "🎮 Master the algorithms and systems that govern robot behavior and decision-making processes. Dive into PID controllers, state machines, and advanced AI-driven control strategies for smooth, intelligent robot operation.",
      icon: "🎮",
      link: "/docs/humanoid-robotics-course/control-systems",
      color: "#f59e0b",
      duration: "30 min read",
      level: "Intermediate",
      topics: ["PID Control", "State Machines", "AI Control", "Feedback Systems"]
    },
    {
      title: "05 - Path Planning and Navigation",
      description: "🗺️ Understand how robots navigate complex environments and plan optimal paths to their goals. Learn about SLAM algorithms, obstacle avoidance, and the mathematics behind autonomous navigation in dynamic environments.",
      icon: "🗺️",
      link: "/docs/humanoid-robotics-course/path-planning-and-navigation",
      color: "#10b981",
      duration: "35 min read",
      level: "Advanced",
      topics: ["SLAM", "Path Planning", "Obstacle Avoidance", "Autonomous Navigation"]
    }
  ];

  return (
    <section id="chapters" className={styles.chapterSection}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <Heading as="h2" className={styles.sectionTitle}>
            📚 Complete Course Curriculum
          </Heading>
          <p className={styles.sectionSubtitle}>
            Master humanoid robotics from fundamentals to advanced applications
          </p>
        </div>
        <div className={styles.chapterGrid}>
          {chapters.map((chapter, index) => (
            <Link
              key={index}
              to={chapter.link}
              className={styles.chapterCard}
              style={{ '--card-accent': chapter.color } as React.CSSProperties}>
              <div className={styles.chapterIcon}>
                <span>{chapter.icon}</span>
              </div>
              <div className={styles.chapterContent}>
                <div className={styles.chapterMeta}>
                  <span className={styles.chapterLevel}>{chapter.level}</span>
                  <span className={styles.chapterDuration}>{chapter.duration}</span>
                </div>
                <h3 className={styles.chapterTitle}>{chapter.title}</h3>
                <p className={styles.chapterDescription}>{chapter.description}</p>
                <div className={styles.chapterTopics}>
                  <strong>Key Topics:</strong>
                  <ul>
                    {chapter.topics.map((topic, topicIndex) => (
                      <li key={topicIndex}>{topic}</li>
                    ))}
                  </ul>
                </div>
                <div className={styles.chapterLink}>
                  <span>🚀 Start Chapter</span>
                  <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                    <path d="M5 12h14M12 5l7 7-7 7"/>
                  </svg>
                </div>
              </div>
            </Link>
          ))}
        </div>
      </div>
    </section>
  );
}

function FeaturesSection() {
  const features = [
    {
      title: "🧠 AI-Driven Content",
      description: "Continuously updated with the latest advancements in robotics research and technology. Our AI ensures you're always learning cutting-edge concepts.",
      icon: "🧠"
    },
    {
      title: "📚 Comprehensive Coverage",
      description: "From basic principles to advanced humanoid systems, covering all essential topics. Each chapter builds systematically on previous knowledge.",
      icon: "📚"
    },
    {
      title: "🔧 Practical Examples",
      description: "Real-world applications and hands-on examples to reinforce learning. See theory come alive with practical robotics implementations.",
      icon: "🔧"
    },
    {
      title: "✨ Interactive Learning",
      description: "Engaging content with visualizations and interactive demonstrations. Complex concepts made simple through intuitive explanations.",
      icon: "✨"
    },
    {
      title: "🎯 Progressive Learning Path",
      description: "Carefully structured curriculum that takes you from beginner to advanced. Each chapter has clear prerequisites and learning outcomes.",
      icon: "🎯"
    },
    {
      title: "🌟 Always Up-to-Date",
      description: "Content evolves with the field of robotics. Learn the latest techniques and best practices as they emerge.",
      icon: "🌟"
    }
  ];

  return (
    <section className={styles.featuresSection}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <Heading as="h2" className={styles.sectionTitle}>
            🌟 Why Choose This Robotics Book?
          </Heading>
          <p className={styles.sectionSubtitle}>
            Experience a revolutionary approach to learning robotics
          </p>
        </div>
        <div className={styles.featuresGrid}>
          {features.map((feature, index) => (
            <div key={index} className={styles.featureCard}>
              <div className={styles.featureIcon}>
                <span>{feature.icon}</span>
              </div>
              <h3 className={styles.featureTitle}>{feature.title}</h3>
              <p className={styles.featureDescription}>{feature.description}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function WelcomeSection() {
  return (
    <section className={styles.welcomeSection}>
      <div className="container">
        <div className={styles.welcomeContent}>
          <Heading as="h2" className={styles.welcomeTitle}>
            🎉 Your Journey Starts Here!
          </Heading>
          <div className={styles.welcomeText}>
            <p>
              <strong>Welcome, future robotics engineer!</strong> You're about to embark on an exciting 
              adventure into one of the most fascinating fields of modern technology. Whether you're 
              taking your first steps into robotics or looking to deepen your expertise, this comprehensive 
              guide is designed to transform your understanding and skills.
            </p>
            <p>
              <strong>What makes this journey special?</strong> This isn't just another textbook – 
              it's a living, breathing resource that grows with the field. Each chapter has been carefully 
              crafted by AI to provide you with the most current, relevant, and practical knowledge available.
            </p>
            <p>
              <strong>Ready to begin?</strong> Start with Chapter 1 and work your way through, or jump 
              to any chapter that matches your current level. Each module is self-contained but builds 
              upon a foundation of shared principles.
            </p>
          </div>
          <div className={styles.welcomeActions}>
            <Link
              className={clsx('button button--primary button--lg', styles.welcomeButton)}
              to="/docs/humanoid-robotics-course/introduction-to-humanoid-robotics">
              🚀 Start Learning Now
            </Link>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): React.ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title} - AI-Driven Robotics Book`}
      description="An innovative, AI-driven comprehensive guide to Physical & Humanoid Robotics">
      <HeroSection />
      <main>
        <WelcomeSection />
        <ChapterSection />
        <FeaturesSection />
      </main>
    </Layout>
  );
}
