import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

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

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <HomepageHeader />
      <main>
        <div className="container">
          <div className="row">
            <div className="col col--8 col--offset-2">
              <Heading as="h1" className={styles.heroTitle}>
                Physical AI & Humanoid Robotics
              </Heading>
              <p className={styles.heroText}>
                Welcome to the AI-native textbook for the Panaversity course. Explore the modules: ROS 2, Gazebo & Unity, NVIDIA Isaac, Vision-Language-Action (VLA), and the Capstone Project in the sidebar.
              </p>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}
