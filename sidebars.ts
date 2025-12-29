import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Physical AI & Humanoid Robotics',
      collapsible: false,
      items: [
        'intro',
        {
          type: 'category',
          label: 'Introduction to Physical AI & Humanoid Robotics',
          collapsed: false,
          items: [
            'introduction/overview',
            'introduction/prerequisites',
            'introduction/course-structure',
            'introduction/learning-outcomes',
          ],
        },
        {
          type: 'category',
          label: 'Module 1: ROS 2 - The Robotic Nervous System',
          collapsed: false,
          items: [
            'ros2/intro',
            'ros2/installation',
            'ros2/basic-concepts',
            'ros2/nodes-topics-services',
            'ros2/packages-workspaces',
            'ros2/practical-examples',
          ],
        },
        {
          type: 'category',
          label: 'Module 2: Digital Twin (Gazebo & Unity)',
          collapsed: false,
          items: [
            'modules/digital-twin/intro',
            'modules/digital-twin/gazebo-installation',
            'modules/digital-twin/gazebo-robot-modeling',
            'modules/digital-twin/gazebo-simulation-basics',
            'modules/digital-twin/gazebo-physics-engines',
            'modules/digital-twin/unity-integration',
            'modules/digital-twin/practical-examples',
          ],
        },
        {
          type: 'category',
          label: 'Module 3: NVIDIA Isaac Platform',
          collapsed: false,
          items: [
            'nvidia-isaac/intro',
            'nvidia-isaac/installation',
            'nvidia-isaac/isaac-ros',
            'nvidia-isaac/isaac-sim',
            'nvidia-isaac/ai-perception',
            'nvidia-isaac/manipulation',
          ],
        },
        {
          type: 'category',
          label: 'Module 4: Vision-Language-Action Models',
          collapsed: false,
          items: [
            'vla/intro',
            'vla/architecture',
            'vla/training-methods',
            'vla/implementation',
            'vla/applications',
            'vla/future-directions',
          ],
        },
        {
          type: 'category',
          label: 'Integration & Advanced Topics',
          collapsed: false,
          items: [
            'integration/hardware',
            'integration/software',
            'integration/real-world-applications',
          ],
        },
        {
          type: 'category',
          label: 'Projects & Case Studies',
          collapsed: false,
          items: [
            'projects/project-ideas',
            'projects/case-studies',
            'projects/evaluation-metrics',
          ],
        },
        'conclusion',
      ],
    },
  ],
};

export default sidebars;
