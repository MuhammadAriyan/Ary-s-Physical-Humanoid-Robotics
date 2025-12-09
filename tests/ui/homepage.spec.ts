import { test, expect } from '@playwright/test';

test.describe('Homepage UI Tests', () => {
  test.beforeEach(async ({ page }) => {
    await page.goto('/');
  });

  test('should load homepage correctly', async ({ page }) => {
    await expect(page).toHaveTitle(/Ary's Physical & Humanoid Robotics/);
    
    // Check if main sections are visible
    await expect(page.locator('.hero')).toBeVisible();
    await expect(page.locator('.features')).toBeVisible();
    await expect(page.locator('.chapters')).toBeVisible();
  });

  test('should have working navigation', async ({ page }) => {
    // Test navbar logo
    const logo = page.locator('.navbar-logo');
    await expect(logo).toBeVisible();
    await expect(logo).toContainText('Ary\'s Robotics');
    
    // Test navigation links
    const bookLink = page.locator('.navbar-link').first();
    await expect(bookLink).toBeVisible();
    await expect(bookLink).toContainText('Book');
    
    // Test theme toggle
    const themeToggle = page.locator('.theme-toggle');
    await expect(themeToggle).toBeVisible();
    await themeToggle.click();
    
    // Check if theme attribute changed
    const html = page.locator('html');
    await expect(html).toHaveAttribute('data-theme', 'dark');
  });

  test('should have responsive hero section', async ({ page }) => {
    const heroTitle = page.locator('.hero-title');
    await expect(heroTitle).toBeVisible();
    await expect(heroTitle).toContainText('Ary\'s Physical & Humanoid Robotics');
    
    const heroSubtitle = page.locator('.hero-subtitle');
    await expect(heroSubtitle).toBeVisible();
    
    // Test CTA buttons
    const startLearningBtn = page.locator('.btn-primary');
    await expect(startLearningBtn).toBeVisible();
    await expect(startLearningBtn).toContainText('Start Learning');
    
    const viewChaptersBtn = page.locator('.btn-secondary');
    await expect(viewChaptersBtn).toBeVisible();
    await expect(viewChaptersBtn).toContainText('View Chapters');
  });

  test('should display features correctly', async ({ page }) => {
    // Scroll to features section
    await page.locator('#features').scrollIntoViewIfNeeded();
    
    const featuresTitle = page.locator('.features-title');
    await expect(featuresTitle).toBeVisible();
    await expect(featuresTitle).toContainText('Why Choose This Robotics Book?');
    
    // Check feature cards
    const featureCards = page.locator('.feature-card');
    await expect(featureCards).toHaveCount(6);
    
    // Test first feature card
    const firstCard = featureCards.first();
    await expect(firstCard.locator('.feature-icon')).toBeVisible();
    await expect(firstCard.locator('.feature-title')).toBeVisible();
    await expect(firstCard.locator('.feature-description')).toBeVisible();
  });

  test('should display chapters correctly', async ({ page }) => {
    // Scroll to chapters section
    await page.locator('#chapters').scrollIntoViewIfNeeded();
    
    const chaptersTitle = page.locator('.chapters-title');
    await expect(chaptersTitle).toBeVisible();
    await expect(chaptersTitle).toContainText('Complete Course Curriculum');
    
    // Check chapter cards
    const chapterCards = page.locator('.chapter-card');
    await expect(chapterCards).toHaveCount(6);
    
    // Test first chapter card
    const firstChapter = chapterCards.first();
    await expect(firstChapter.locator('.chapter-icon')).toBeVisible();
    await expect(firstChapter.locator('.chapter-title')).toBeVisible();
    await expect(firstChapter.locator('.chapter-description')).toBeVisible();
    
    // Test chapter metadata
    const chapterMeta = firstChapter.locator('.chapter-meta');
    await expect(chapterMeta).toBeVisible();
    
    const chapterTags = firstChapter.locator('.chapter-tag');
    await expect(chapterTags).toHaveCount(2);
  });

  test('should have working links', async ({ page }) => {
    // Test "Start Learning" button
    const startLearningBtn = page.locator('.btn-primary');
    await startLearningBtn.click();
    await expect(page).toHaveURL(/.*docs\/humanoid-robotics-course\/introduction-to-humanoid-robotics/);
  });

  test('should be mobile responsive', async ({ page }) => {
    // Test mobile viewport
    await page.setViewportSize({ width: 375, height: 667 });
    
    // Check mobile navigation
    const navbar = page.locator('.navbar');
    await expect(navbar).toBeVisible();
    
    // Check mobile hero
    const heroTitle = page.locator('.hero-title');
    await expect(heroTitle).toBeVisible();
    
    // Check mobile buttons
    const heroActions = page.locator('.hero-actions');
    await expect(heroActions).toBeVisible();
    
    const buttons = heroActions.locator('.btn');
    await expect(buttons).toHaveCount(2);
  });

  test('should have smooth scrolling', async ({ page }) => {
    // Test smooth scroll to features
    const featuresLink = page.locator('a[href="#features"]');
    await featuresLink.click();
    
    // Check if features section is in view
    const features = page.locator('#features');
    await expect(features).toBeInViewport();
    
    // Test smooth scroll to chapters
    const chaptersLink = page.locator('a[href="#chapters"]');
    await chaptersLink.click();
    
    // Check if chapters section is in view
    const chapters = page.locator('#chapters');
    await expect(chapters).toBeInViewport();
  });

  test('should have proper hover effects', async ({ page }) => {
    // Test button hover effects
    const primaryBtn = page.locator('.btn-primary');
    await primaryBtn.hover();
    await expect(primaryBtn).toHaveCSS('transform', 'matrix(1, 0, 0, 1, 0, -2)');
    
    // Test card hover effects
    const featureCard = page.locator('.feature-card').first();
    await featureCard.hover();
    await expect(featureCard).toHaveCSS('transform', 'matrix(1, 0, 0, 1, 0, -4)');
    
    const chapterCard = page.locator('.chapter-card').first();
    await chapterCard.hover();
    await expect(chapterCard).toHaveCSS('transform', 'matrix(1, 0, 0, 1, 0, -4)');
  });

  test('should have proper animations', async ({ page }) => {
    // Check if elements have animation classes
    const heroContent = page.locator('.hero-content');
    await expect(heroContent).toHaveClass(/animate-fade-in-up/);
    
    const featureCards = page.locator('.feature-card');
    const count = await featureCards.count();
    
    for (let i = 0; i < count; i++) {
      const card = featureCards.nth(i);
      await expect(card).toHaveClass(/animate-fade-in-up/);
    }
  });
});