// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['intro'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Chapter 1: Foundations of ROS 2',
      items: [
        'chapter-1/index',
        'chapter-1/concepts',
        'chapter-1/middleware',
        'chapter-1/distributed',
        'chapter-1/validation'
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Chapter 2: ROS 2 Communication',
      items: [
        'chapter-2/index',
        'chapter-2/nodes',
        'chapter-2/topics',
        'chapter-2/services',
        'chapter-2/validation'
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Chapter 3: Python Integration',
      items: [
        'chapter-3/index',
        'chapter-3/rclpy-architecture',
        'chapter-3/python-nodes',
        'chapter-3/rclpy-operations',
        'chapter-3/python-agent-example',
        'chapter-3/validation'
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Chapter 4: URDF for Humanoids',
      items: [
        'chapter-4/index',
        'chapter-4/urdf-concepts',
        'chapter-4/kinematic-chains',
        'chapter-4/humanoid-models',
        'chapter-4/validation'
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'chapter-5-digital-twin/index',
        'chapter-5-digital-twin/concepts',
        'chapter-5-digital-twin/physics-simulation',
        'chapter-5-digital-twin/validation',
        'chapter-5-digital-twin/impact-assessment',
        'chapter-6-gazebo/index',
        'chapter-6-gazebo/setup',
        'chapter-6-gazebo/physics-engine',
        'chapter-6-gazebo/sensor-simulation',
        'chapter-6-gazebo/learning-outcomes',
        'chapter-7-unity/index',
        'chapter-7-unity/integration',
        'chapter-7-unity/environment-modeling',
        'chapter-7-unity/rendering',
        'chapter-7-unity/platform-consistency',
        'chapter-8-sync/index',
        'chapter-8-sync/sync-protocols',
        'chapter-8-sync/validation-frameworks',
        'chapter-8-sync/roi-assessment',
        'chapter-8-sync/academic-rigor',
        'module-2-conclusion'
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 3: Advanced Platforms & AI Integration',
      items: [
        {
          type: 'category',
          label: 'Chapter 9: Isaac Platform Architecture',
          items: [
            'chapter-9-isaac-platform-architecture/index',
            'chapter-9-isaac-platform-architecture/isaac-ecosystem',
            'chapter-9-isaac-platform-architecture/integration-architecture',
            'chapter-9-isaac-platform-architecture/educational-applications',
            'chapter-9-isaac-platform-architecture/benefits-considerations',
          ],
          collapsed: false,
        },
        {
          type: 'category',
          label: 'Chapter 10: Photorealistic Simulation',
          items: [
            'chapter-10-photorealistic-simulation/index',
            'chapter-10-photorealistic-simulation/simulation-fidelity',
            'chapter-10-photorealistic-simulation/synthetic-data-generation',
            'chapter-10-photorealistic-simulation/perception-pipeline-architectures',
            'chapter-10-photorealistic-simulation/sim-to-real-transfer',
            'chapter-10-photorealistic-simulation/validation-testing',
          ],
          collapsed: false,
        },
        {
          type: 'category',
          label: 'Chapter 11: Navigation Path Planning',
          items: [
            'chapter-11-navigation-path-planning/index',
            'chapter-11-navigation-path-planning/nav2-integration',
            'chapter-11-navigation-path-planning/path-planning-algorithms',
            'chapter-11-navigation-path-planning/localization-mapping',
            'chapter-11-navigation-path-planning/dynamic-environment-navigation',
            'chapter-11-navigation-path-planning/multi-robot-navigation',
          ],
          collapsed: false,
        },
        {
          type: 'category',
          label: 'Chapter 12: Perception Action Integration',
          items: [
            'chapter-12-perception-action-integration/index',
            'chapter-12-perception-action-integration/pipeline-architecture',
            'chapter-12-perception-action-integration/integration-components',
            'chapter-12-perception-action-integration/deployment-strategies',
            'chapter-12-perception-action-integration/evaluation-validation',
            'chapter-12-perception-action-integration/integration-challenges',
          ],
          collapsed: false,
        }
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        {
          type: 'category',
          label: 'Chapter 13: Language Perception in VLA Systems',
          items: [
            'chapter-13-language-perception/index',
            'chapter-13-language-perception/language-perception',
            'chapter-13-language-perception/language-understanding-component',
            'chapter-13-language-perception/speech-to-text-integration',
            'chapter-13-language-perception/llm-integration-patterns',
            'chapter-13-language-perception/llm-robotics-convergence',
            'chapter-13-language-perception/llm-robotics-convergence-diagrams'
          ],
          collapsed: false,
        },
        {
          type: 'category',
          label: 'Chapter 14: Cognitive Planning and Decision Making',
          items: [
            'chapter-14-cognitive-planning/cognitive-planning',
            'chapter-14-cognitive-planning/cognitive-planning-component',
            'chapter-14-cognitive-planning/vla-architecture-concepts',
            'chapter-14-cognitive-planning/vla-architecture-diagrams',
            'chapter-14-cognitive-planning/vla-system-components'
          ],
          collapsed: false,
        },
        {
          type: 'category',
          label: 'Chapter 15: Action Execution and Control',
          items: [
            'chapter-15-action-execution/action-execution',
            'chapter-15-action-execution/physical-execution-component',
            'chapter-15-action-execution/perception-action-integration',
            'chapter-15-action-execution/ros2-integration-patterns'
          ],
          collapsed: false,
        },
        {
          type: 'category',
          label: 'Chapter 16: System Integration and Autonomous Humanoid Capstone',
          items: [
            'chapter-16-system-integration/system-integration',
            'chapter-16-system-integration/autonomous-humanoid-capstone-summary',
            'chapter-16-system-integration/vla-educational-applications',
            'chapter-16-system-integration/vla-educational-value-proposition',
            'chapter-16-system-integration/vla-module-deployment-integration-guide'
          ],
          collapsed: false,
        }
      ],
      collapsed: false,
    }
  ],
};

export default sidebars;