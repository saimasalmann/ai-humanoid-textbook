// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  textbookSidebar: [
    {
      type: 'doc',
      id: 'intro',
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 as the Robotic Nervous System',
      items: [
        'module-1-ros/index',
        'module-1-ros/ros-fundamentals',
        'module-1-ros/building-nodes',
        'module-1-ros/ros2-python',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twins with Gazebo & Unity',
      items: [
        'module-2-simulation/index',
        'module-2-simulation/gazebo-intro',
        'module-2-simulation/urdf-modeling',
        'module-2-simulation/unity-visualization',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 3: AI-Robot Brain with NVIDIA Isaac',
      items: [
        'module-3-isaac/index',
        'module-3-isaac/isaac-overview',
        'module-3-isaac/perception-stack',
        'module-3-isaac/rl-navigation',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action Systems',
      items: [
        'module-4-vla/index',
        'module-4-vla/vision-systems',
        'module-4-vla/language-integration',
        'module-4-vla/action-planning',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Capstone: Autonomous Conversational Humanoid Robot',
      items: [
        'capstone/index',
        'capstone/architecture',
        'capstone/implementation',
        'capstone/evaluation',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Reference',
      items: [
        'reference/glossary',
        'reference/api-reference',
        'reference/troubleshooting',
      ],
      collapsed: true,
    },
    {
      type: 'category',
      label: 'Tutorials',
      items: [
        'tutorials/setup-guide',
        'tutorials/first-robot',
        'tutorials/debugging-tips',
      ],
      collapsed: true,
    },
  ],
};

export default sidebars;