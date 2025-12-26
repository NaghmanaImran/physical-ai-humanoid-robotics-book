/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  moduleSidebar: [
    {
      type: 'category',
      label: 'Module 1: ROS 2 - The Robotic Nervous System',
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

module.exports = sidebars;