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
          <p className={styles.heroText}>
            Welcome to the AI-native textbook for the Panaversity course. Explore the modules: ROS 2, Gazebo & Unity, NVIDIA Isaac, Vision-Language-Action (VLA), and the Capstone Project in the sidebar.
          </p>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Physical AI & Humanoid Robotics`}
      description="A Comprehensive Course on Embodied Intelligence and Humanoid Robots">
      <HomepageHeader />
    </Layout>
  );
}
