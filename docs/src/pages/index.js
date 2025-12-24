import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';

import Heading from '@theme/Heading';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={styles.heroBanner}>
      <div className="container">
        <div className={styles.heroContent}>
          <Heading as="h1" className={styles.heroTitle}>
            Physical AI & Humanoid Robotics
          </Heading>
          <Heading as="h2" className={styles.heroSubtitle}>
            Panaversity AI-Native Book Series
          </Heading>
          <p className={styles.heroTagline}>
            AI Native Software Development<br/>
            Co-Learning Agentic AI with Python and TypeScript – Spec-Driven Reusable Intelligence
          </p>
          <div className={styles.heroButtons}>
            <Link
              className="button button--primary button--lg"
              to="/docs/intro">
              Start Reading
            </Link>
            <Link
              className="button button--secondary button--lg"
              to="/docs/tutorial">
              View Tutorials
            </Link>
          </div>
        </div>
      </div>
    </header>
  );
}

function FeatureGrid() {
  const features = [
    {
      title: 'Open Source',
      icon: '✨',
      description: 'Fully open source content and tools for collaborative development.'
    },
    {
      title: 'Co-Learning with AI',
      icon: '🤝',
      description: 'Learn alongside AI agents that enhance your development process.'
    },
    {
      title: 'Spec-Driven Development',
      icon: '🎯',
      description: 'Build software with clear specifications and testable requirements.'
    },
    {
      title: 'AI-Native Architecture',
      icon: '🧠',
      description: 'Design systems that leverage AI capabilities from the ground up.'
    }
  ];

  return (
    <section className={styles.features}>
      <div className="container">
        <div className={styles.featuresGrid}>
          {features.map((feature, idx) => (
            <div className={styles.featureCard} key={idx}>
              <div className={styles.featureIcon}>{feature.icon}</div>
              <h3 className={styles.featureTitle}>{feature.title}</h3>
              <p className={styles.featureDescription}>{feature.description}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Physical AI & Humanoid Robotics`}
      description="Panaversity AI-Native Book Series - AI Native Software Development Co-Learning Agentic AI with Python and TypeScript – Spec-Driven Reusable Intelligence">
      <HomepageHeader />
      <FeatureGrid />
    </Layout>
  );
}
