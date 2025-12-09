import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// Manual sidebar configuration for better organization
const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Course Content',
      items: [
        'humanoid-robotics-course/introduction-to-humanoid-robotics',
        'humanoid-robotics-course/sensors-and-perception',
        'humanoid-robotics-course/actuators-and-movement',
        'humanoid-robotics-course/control-systems',
        'humanoid-robotics-course/path-planning-and-navigation',
      ],
    },
    {
      type: 'doc',
      id: 'humanoid-robotics-course/textbook-outline',
      label: 'Textbook Outline',
    },
    
  ],
};

export default sidebars;