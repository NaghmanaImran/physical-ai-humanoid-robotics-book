import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Beginner-Friendly Learning',
    Svg: require('@site/static/img/undraw_robot_learning.svg').default,
    description: (
      <>
        Our Physical AI & Robotics curriculum is designed from the ground up to be easily understood and
        applied to get you building robots quickly. Start with ROS 2 basics and progress to advanced humanoid systems.
      </>
    ),
  },
  {
    title: 'Focus on What Matters',
    Svg: require('@site/static/img/undraw_robotics_research.svg').default,
    description: (
      <>
        Focus on hands-on robotics learning with simulations and real-world applications. We handle the complex setup -
        you focus on mastering Physical AI, robotics frameworks, and humanoid systems in our structured curriculum.
      </>
    ),
  },
  {
    title: 'Powered by React',
    Svg: require('@site/static/img/undraw_ai_platform.svg').default,
    description: (
      <>
        Our interactive learning platform leverages React for responsive UI, performance, and extensibility. 
        The platform can be enhanced with custom components while maintaining consistent educational experiences.
      </>
    ),
  },
];

function Feature({title, Svg, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}