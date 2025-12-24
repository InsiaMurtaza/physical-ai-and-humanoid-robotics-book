import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className={styles.backgroundElements}>
        <div className={styles.gridPattern}></div>
        <div className={styles.roboticElements}>
          {/* Abstract humanoid silhouettes */}
          <svg className={styles.humanoidSilhouette} width="200" height="400" viewBox="0 0 100 200" style={{top: '10%', left: '5%'}}>
            <circle cx="50" cy="30" r="15"/>
            <line x1="50" y1="45" x2="50" y2="120" />
            <line x1="35" y1="70" x2="65" y2="70" />
            <line x1="50" y1="120" x2="35" y2="160" />
            <line x1="50" y1="120" x2="65" y2="160" />
          </svg>

          <svg className={styles.humanoidSilhouette} width="150" height="350" viewBox="0 0 100 200" style={{top: '15%', right: '7%'}}>
            <circle cx="50" cy="30" r="15"/>
            <line x1="50" y1="45" x2="50" y2="120" />
            <line x1="35" y1="70" x2="65" y2="70" />
            <line x1="50" y1="120" x2="35" y2="160" />
            <line x1="50" y1="120" x2="65" y2="160" />
          </svg>

          {/* Neural network pattern */}
          <svg className={styles.neuralNetwork} width="400" height="250" viewBox="0 0 400 250" style={{top: '25%', left: '15%'}}>
            {/* Main neural network nodes */}
            <circle cx="50" cy="50" r="6" fill="#60a5fa"/>
            <circle cx="150" cy="40" r="6" fill="#8b5cf6"/>
            <circle cx="250" cy="60" r="6" fill="#ec4899"/>
            <circle cx="100" cy="120" r="6" fill="#34d399"/>
            <circle cx="200" cy="140" r="6" fill="#fbbf24"/>
            <circle cx="80" cy="180" r="6" fill="#60a5fa"/>
            <circle cx="220" cy="100" r="6" fill="#ec4899"/>
            <circle cx="300" cy="90" r="6" fill="#8b5cf6"/>
            <circle cx="350" cy="150" r="6" fill="#34d399"/>

            {/* Connections */}
            <line x1="50" y1="50" x2="150" y2="40" stroke="#60a5fa" strokeWidth="2"/>
            <line x1="150" y1="40" x2="250" y2="60" stroke="#8b5cf6" strokeWidth="2"/>
            <line x1="50" y1="50" x2="100" y2="120" stroke="#34d399" strokeWidth="2"/>
            <line x1="100" y1="120" x2="80" y2="180" stroke="#fbbf24" strokeWidth="2"/>
            <line x1="100" y1="120" x2="200" y2="140" stroke="#ec4899" strokeWidth="2"/>
            <line x1="200" y1="140" x2="220" y2="100" stroke="#60a5fa" strokeWidth="2"/>
            <line x1="220" y1="100" x2="300" y2="90" stroke="#ec4899" strokeWidth="2"/>
            <line x1="300" y1="90" x2="350" y2="150" stroke="#34d399" strokeWidth="2"/>
            <line x1="250" y1="60" x2="300" y2="90" stroke="#8b5cf6" strokeWidth="2"/>
            <line x1="200" y1="140" x2="350" y2="150" stroke="#fbbf24" strokeWidth="2"/>
          </svg>

          {/* Additional robotics elements */}
          <svg className={styles.humanoidSilhouette} width="120" height="280" viewBox="0 0 80 180" style={{bottom: '10%', left: '10%'}}>
            {/* Head */}
            <circle cx="40" cy="25" r="12" stroke="#60a5fa" strokeWidth="2" fill="none"/>
            {/* Body */}
            <line x1="40" y1="37" x2="40" y2="95" stroke="#60a5fa" strokeWidth="2"/>
            {/* Arms */}
            <line x1="25" y1="60" x2="55" y2="60" stroke="#60a5fa" strokeWidth="2"/>
            {/* Legs */}
            <line x1="40" y1="95" x2="30" y2="140" stroke="#60a5fa" strokeWidth="2"/>
            <line x1="40" y1="95" x2="50" y2="140" stroke="#60a5fa" strokeWidth="2"/>
            {/* Sensors on head */}
            <circle cx="35" cy="20" r="2" fill="#ec4899"/>
            <circle cx="45" cy="20" r="2" fill="#ec4899"/>
          </svg>

          <svg className={styles.humanoidSilhouette} width="100" height="250" viewBox="0 0 70 160" style={{bottom: '15%', right: '12%'}}>
            {/* Head */}
            <circle cx="35" cy="20" r="10" stroke="#8b5cf6" strokeWidth="2" fill="none"/>
            {/* Body */}
            <rect x="25" y="30" width="20" height="50" stroke="#8b5cf6" strokeWidth="2" fill="none"/>
            {/* Arms */}
            <line x1="20" y1="45" x2="50" y2="45" stroke="#8b5cf6" strokeWidth="2"/>
            {/* Legs */}
            <line x1="30" y1="80" x2="25" y2="120" stroke="#8b5cf6" strokeWidth="2"/>
            <line x1="40" y1="80" x2="45" y2="120" stroke="#8b5cf6" strokeWidth="2"/>
            {/* Circuit pattern on body */}
            <rect x="28" y="35" width="14" height="10" stroke="#fbbf24" strokeWidth="1" fill="none"/>
            <circle cx="35" cy="50" r="3" stroke="#fbbf24" strokeWidth="1" fill="none"/>
          </svg>

          {/* Sensor dots */}
          {[...Array(25)].map((_, i) => (
            <div
              key={i}
              className={styles.sensorDot}
              style={{
                top: `${Math.random() * 100}%`,
                left: `${Math.random() * 100}%`,
              }}
            ></div>
          ))}

          {/* Joint circles */}
          {[...Array(15)].map((_, i) => (
            <div
              key={i}
              className={styles.jointCircle}
              style={{
                top: `${Math.random() * 100}%`,
                left: `${Math.random() * 100}%`,
              }}
            ></div>
          ))}

          {/* Circuit board pattern */}
          <svg className={styles.neuralNetwork} width="200" height="150" viewBox="0 0 200 150" style={{top: '40%', right: '20%'}}>
            {/* Circuit paths */}
            <path d="M20,30 L80,30 L80,60 L120,60 L120,90" stroke="#34d399" strokeWidth="2" fill="none"/>
            <path d="M40,100 L70,100 L70,70 L110,70 L110,40" stroke="#fbbf24" strokeWidth="2" fill="none"/>
            <path d="M150,20 L150,50 L180,50" stroke="#ec4899" strokeWidth="2" fill="none"/>
            <path d="M150,80 L150,110 L170,110" stroke="#60a5fa" strokeWidth="2" fill="none"/>

            {/* Circuit components */}
            <rect x="15" y="25" width="10" height="10" stroke="#34d399" strokeWidth="1" fill="none"/>
            <circle cx="75" cy="55" r="5" stroke="#fbbf24" strokeWidth="1" fill="none"/>
            <rect x="115" y="85" width="10" height="10" stroke="#ec4899" strokeWidth="1" fill="none"/>
            <circle cx="145" cy="15" r="5" stroke="#60a5fa" strokeWidth="1" fill="none"/>
            <circle cx="145" cy="75" r="5" stroke="#8b5cf6" strokeWidth="1" fill="none"/>
          </svg>
        </div>
      </div>

      <div className={styles.heroContainer}>
        <h1 className={clsx('hero__title', styles.title)}>
          Physical AI & Humanoid Robotics Course
        </h1>
        <p className={clsx('hero__subtitle', styles.subtitle)}>
          A comprehensive guide to embodied intelligence in physical environments
        </p>
        <p className={styles.tagline}>
          Bridging the gap between digital AI systems and their physical manifestations,
          providing you with the knowledge and skills needed to develop intelligent robots
          that interact with the real world.
        </p>
        <div className={styles.buttons}>
          <Link
            className={clsx('button button--secondary button--lg', styles.button, styles.primaryButton)}
            to="/docs/intro">
            Start Reading
          </Link>
        </div>
      </div>
    </header>
  );
}

function FeatureSection() {
  const features = [
    {
      title: 'Physical AI Principles',
      description: 'Understand the fundamentals of embodied intelligence and how AI systems operate in physical environments.',
    },
    {
      title: 'Humanoid Robotics',
      description: 'Learn to design and control humanoid robots that can navigate and interact in human-centered environments.',
    },
    {
      title: 'Simulation & Control',
      description: 'Master ROS 2, Gazebo, Unity, and NVIDIA Isaac platforms for robot simulation and control.',
    },
    {
      title: 'Conversational AI',
      description: 'Integrate advanced language models for natural human-robot interaction and communication.',
    },
    {
      title: 'Sensor Integration',
      description: 'Work with LIDAR, cameras, IMUs, and force/torque sensors for environmental awareness.',
    },
    {
      title: 'Real-World Applications',
      description: 'Apply learned concepts to practical scenarios and capstone projects.',
    },
  ];

  return (
    <section className={styles.contentSection}>
      <div className="container">
        <h2 style={{fontSize: '2.5rem', marginBottom: '1rem', color: '#000000'}}>Course Highlights</h2>
        <p style={{fontSize: '1.25rem', color: '#000000', maxWidth: '700px', margin: '0 auto 3rem'}}>
          Master the essential concepts and technologies that power the next generation of intelligent robots
        </p>

        <div className={styles.featuresGrid}>
          {features.map((feature, idx) => (
            <div key={idx} className={styles.featureCard}>
              <div className={styles.featureIcon}>🤖</div>
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
      description="A comprehensive guide to Physical AI & Humanoid Robotics">
      <HomepageHeader />
      <main>
        <FeatureSection />
      </main>
    </Layout>
  );
}