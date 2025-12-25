import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// Physical AI & Humanoid Robotics Book Navigation
const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Part 1: Foundations (Weeks 1-2)',
      collapsible: true,
      collapsed: false,
      items: [
        'part-1-foundations/introduction-to-physical-ai',
        'part-1-foundations/01a-week-1-2-overview',
      ],
    },
    {
      type: 'category',
      label: 'Part 2: ROS 2 Fundamentals (Weeks 3-5)',
      collapsible: true,
      collapsed: true,
      items: [
        'part-2-ros2/ros2-fundamentals',
        'part-2-ros2/02a-week-3-5-overview',
      ],
    },
    {
      type: 'category',
      label: 'Part 3: Simulation (Weeks 6-7)',
      collapsible: true,
      collapsed: true,
      items: [
        'part-3-simulation/gazebo-unity-simulation',
        'part-3-simulation/03a-week-6-7-overview',
      ],
    },
    {
      type: 'category',
      label: 'Part 4: NVIDIA Isaac (Weeks 8-10)',
      collapsible: true,
      collapsed: true,
      items: [
        'part-4-isaac/nvidia-isaac-platform',
        'part-4-isaac/04a-week-8-10-overview',
      ],
    },
    {
      type: 'category',
      label: 'Part 5: Humanoid Development (Weeks 11-12)',
      collapsible: true,
      collapsed: true,
      items: [
        'part-5-humanoid/humanoid-robot-development',
        'part-5-humanoid/05a-week-11-12-overview',
      ],
    },
    {
      type: 'category',
      label: 'Part 6: Conversational Robotics (Week 13)',
      collapsible: true,
      collapsed: true,
      items: [
        'part-6-conversational/conversational-robotics',
        'part-6-conversational/06a-week-13-overview',
      ],
    },
    {
      type: 'category',
      label: 'Appendices',
      collapsible: true,
      collapsed: true,
      items: [
        'appendix/hardware-specifications',
        'appendix/B-simulation-setup',
        'appendix/C-community-resources',
        'appendix/D-assessment-rubrics',
      ],
    },
  ],
};

export default sidebars;