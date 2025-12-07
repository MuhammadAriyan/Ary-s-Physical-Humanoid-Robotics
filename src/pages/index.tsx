import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageIntroduction() {
  return (
    <section className={clsx('hero', styles.heroIntroduction)}>
      <div className="container">
        <p className="hero__subtitle">
          Welcome to "Ary's Physical & Humanoid Robotics," an innovative, AI-driven book designed to take you on a journey from the fundamental principles of robotics to the advanced concepts of humanoid systems. This book is automatically generated and updated to provide you with the most current and comprehensive information in the field.
        </p>
      </div>
    </section>
  );
}

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
      </div>
    </header>
  );
}

function ChapterNavigation() {
  return (
    <section className={styles.chapters}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <h2>Chapters</h2>
            <div className={styles.buttons}>
              <Link
                className="button button--secondary button--lg"
                to="/docs/intro-to-robotics">
                Chapter 1: Introduction to Robotics
              </Link>
            </div>
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
      title={`Welcome to ${siteConfig.title}`}
      description="An AI-driven book on Physical & Humanoid Robotics">
      <HomepageHeader />
      <main>
        <HomepageIntroduction />
        <ChapterNavigation />
      </main>
    </Layout>
  );
}
