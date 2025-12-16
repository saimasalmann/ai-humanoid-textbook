import React from 'react';
import clsx from 'clsx';
import styles from './HomepageFeatures.module.css';

const FeatureList = [
  {
    title: 'Physical AI & Robotics',
    Svg: require('@site/static/img/robotics.svg').default,
    description: (
      <>
        Learn about the intersection of artificial intelligence and physical robotics,
        including embodied intelligence and real-world interaction systems.
      </>
    ),
  },
  {
    title: 'ROS 2 Fundamentals',
    Svg: require('@site/static/img/ros2.svg').default,
    description: (
      <>
        Master Robot Operating System 2 (ROS 2) for building distributed robotics applications.
      </>
    ),
  },
  {
    title: 'Simulation to Reality',
    Svg: require('@site/static/img/simulation.svg').default,
    description: (
      <>
        Bridge the gap between simulation environments and real-world robotics deployment.
      </>
    ),
  },
];

function Feature({Svg, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
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