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
            'introduction/history',
            'introduction/fundamentals',
          ],
        },
        {
          type: 'category',
          label: 'ROS2 Module',
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
          label: 'Gazebo Module',
          collapsed: false,
          items: [
            'gazebo/intro',
            'gazebo/installation',
            'gazebo/simulation-basics',
            'gazebo/robot-modeling',
            'gazebo/physics-engines',
            'gazebo/integration-with-ros2',
          ],
        },
        {
          type: 'category',
          label: 'NVIDIA Isaac Module',
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
          label: 'VLA (Vision-Language-Action) Module',
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
            'integration/training-methods',
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
