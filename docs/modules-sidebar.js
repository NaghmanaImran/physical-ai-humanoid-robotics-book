// @ts-check

/**
 * @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  // Sidebar for the ROS 2 module
  ros2nervoussystem: [
    {
      type: 'category',
      label: 'ROS 2: The Robotic Nervous System',
      collapsible: true,
      collapsed: false,
      items: [
        'ros2-nervous-system/chapter-1-intro',
        'ros2-nervous-system/chapter-2-communication',
        'ros2-nervous-system/chapter-3-structure',
      ],
    },
  ],
};

export default sidebars;