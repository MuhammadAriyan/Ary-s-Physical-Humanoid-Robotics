/**
 * Chapter mapping for documentation navigation
 * Maps chapter IDs to their documentation URLs
 */

export const DOC_CHAPTERS: Record<string, string> = {
  'introduction-to-humanoid-robotics': '/docs/humanoid-robotics-course/introduction-to-humanoid-robotics',
  'sensors-and-perception': '/docs/humanoid-robotics-course/sensors-and-perception',
  'actuators-and-movement': '/docs/humanoid-robotics-course/actuators-and-movement',
  'control-systems': '/docs/humanoid-robotics-course/control-systems',
  'path-planning-and-navigation': '/docs/humanoid-robotics-course/path-planning-and-navigation',
};

export const CHAPTER_TITLES: Record<string, string> = {
  'introduction-to-humanoid-robotics': 'Introduction',
  'sensors-and-perception': 'Sensors & Perception',
  'actuators-and-movement': 'Actuators & Movement',
  'control-systems': 'Control Systems',
  'path-planning-and-navigation': 'Path Planning',
};

export const DEFAULT_CHAPTER = 'introduction-to-humanoid-robotics';

export const VALID_CHAPTERS = Object.keys(DOC_CHAPTERS);
