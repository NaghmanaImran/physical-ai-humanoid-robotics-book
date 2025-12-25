// @ts-check

/**
 * @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  // Sidebar for all modules
  moduleSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      collapsible: true,
      collapsed: false,
      items: [
        'intro',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2',
      collapsible: true,
      collapsed: false,
      items: [
        'ros2/intro',
        'ros2/installation',
        'ros2/packages-workspaces',
        'ros2/nodes-topics-services',
        'ros2/basic-concepts',
        'ros2/practical-examples',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin',
      collapsible: true,
      collapsed: false,
      items: [
        'gazebo/intro',
        'gazebo/installation',
        'gazebo/simulation-basics',
        'gazebo/physics-engines',
        'gazebo/robot-modeling',
        'gazebo/integration-with-ros2',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac',
      collapsible: true,
      collapsed: false,
      items: [
        'nvidia-isaac/intro',
        'nvidia-isaac/installation',
        'nvidia-isaac/isaac-sim',
        'nvidia-isaac/isaac-ros',
        'nvidia-isaac/ai-perception',
        'nvidia-isaac/manipulation',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA',
      collapsible: true,
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
      label: 'Capstone Project',
      collapsible: true,
      collapsed: false,
      items: [
        'capstone-project',
      ],
    },
  ],
};

export default sidebars;