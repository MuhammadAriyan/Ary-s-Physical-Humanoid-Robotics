/**
 * Chapter mapping for documentation navigation
 * Maps chapter IDs to their documentation URLs
 * Now points to the Physical AI & Humanoid Robotics book
 */

export const DOC_CHAPTERS: Record<string, string> = {
  'introduction-to-physical-ai': '/docs/part-1-foundations/introduction-to-physical-ai',
  'ros2-fundamentals': '/docs/part-2-ros2/ros2-fundamentals',
  'simulation': '/docs/part-3-simulation/gazebo-unity-simulation',
  'nvidia-isaac': '/docs/part-4-isaac/nvidia-isaac-platform',
  'humanoid-development': '/docs/part-5-humanoid/humanoid-robot-development',
  'conversational-robotics': '/docs/part-6-conversational/conversational-robotics',
};

export const CHAPTER_TITLES: Record<string, string> = {
  'introduction-to-physical-ai': 'Part 1: Physical AI Foundations',
  'ros2-fundamentals': 'Part 2: ROS 2 Fundamentals',
  'simulation': 'Part 3: Simulation',
  'nvidia-isaac': 'Part 4: NVIDIA Isaac',
  'humanoid-development': 'Part 5: Humanoid Development',
  'conversational-robotics': 'Part 6: Conversational Robotics',
};

export const DEFAULT_CHAPTER = 'introduction-to-physical-ai';

export const VALID_CHAPTERS = Object.keys(DOC_CHAPTERS);
