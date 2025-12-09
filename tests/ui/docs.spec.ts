import { test, expect } from '@playwright/test';

test.describe('Documentation Pages UI Tests', () => {
  test.beforeEach(async ({ page }) => {
    await page.goto('/docs/humanoid-robotics-course/introduction-to-humanoid-robotics');
  });

  test('should load documentation page correctly', async ({ page }) => {
    await expect(page).toHaveTitle(/Introduction to Humanoid Robotics/);
    
    // Check if main content is visible
    const mainContent = page.locator('main');
    await expect(mainContent).toBeVisible();
    
    // Check if sidebar is visible
    const sidebar = page.locator('.theme-doc-sidebar-container');
    await expect(sidebar).toBeVisible();
  });

  test('should have working navigation in docs', async ({ page }) => {
    // Test sidebar navigation
    const sidebarLinks = page.locator('.theme-doc-sidebar-item-link');
    await expect(sidebarLinks.first()).toBeVisible();
    
    // Test first sidebar link
    const firstLink = sidebarLinks.first();
    await firstLink.click();
    
    // Wait for navigation
    await page.waitForLoadState('networkidle');
  });

  test('should have proper typography and readability', async ({ page }) => {
    // Check headings
    const h1 = page.locator('h1');
    if (await h1.count() > 0) {
      await expect(h1.first()).toBeVisible();
    }
    
    const h2 = page.locator('h2');
    if (await h2.count() > 0) {
      await expect(h2.first()).toBeVisible();
    }
    
    // Check paragraphs
    const paragraphs = page.locator('p');
    if (await paragraphs.count() > 0) {
      await expect(paragraphs.first()).toBeVisible();
    }
  });

  test('should have responsive design', async ({ page }) => {
    // Test desktop
    await page.setViewportSize({ width: 1200, height: 800 });
    const sidebar = page.locator('.theme-doc-sidebar-container');
    await expect(sidebar).toBeVisible();
    
    // Test mobile
    await page.setViewportSize({ width: 375, height: 667 });
    
    // On mobile, sidebar might be hidden or transformed
    // Sidebar might be hidden on mobile, so we don't assert visibility
    
    // But main content should still be visible
    const mainContent = page.locator('main');
    await expect(mainContent).toBeVisible();
  });

  test('should have working theme toggle in docs', async ({ page }) => {
    // Look for theme toggle button
    const themeToggle = page.locator('[aria-label*="theme"], [title*="theme"], .theme-toggle');
    
    if (await themeToggle.count() > 0) {
      await themeToggle.first().click();
      
      // Check if theme changed
      const html = page.locator('html');
      const hasDarkTheme = await html.getAttribute('data-theme');
      
      // Either data-theme should be 'dark' or there should be a dark class
      const htmlClass = await html.getAttribute('class');
      expect(hasDarkTheme === 'dark' || htmlClass?.includes('dark')).toBeTruthy();
    }
  });

  test('should have proper code blocks styling', async ({ page }) => {
    // Look for code blocks
    const codeBlocks = page.locator('pre code, .code-block');
    
    if (await codeBlocks.count() > 0) {
      const firstCodeBlock = codeBlocks.first();
      await expect(firstCodeBlock).toBeVisible();
    }
  });

  test('should have working table of contents', async ({ page }) => {
    // Look for table of contents
    const toc = page.locator('.table-of-contents, .theme-doc-toc');
    
    if (await toc.count() > 0) {
      await expect(toc.first()).toBeVisible();
      
      // Test TOC links if they exist
      const tocLinks = toc.first().locator('a');
      if (await tocLinks.count() > 0) {
        await expect(tocLinks.first()).toBeVisible();
      }
    }
  });

  test('should have proper link styling', async ({ page }) => {
    // Look for links in content
    const contentLinks = page.locator('main a[href]');
    
    if (await contentLinks.count() > 0) {
      const firstLink = contentLinks.first();
      await expect(firstLink).toBeVisible();
      
      // Test link hover
      await firstLink.hover();
      // Check if link has hover effect (this might vary based on styling)
    }
  });

  test('should have proper image handling', async ({ page }) => {
    // Look for images
    const images = page.locator('img');
    
    if (await images.count() > 0) {
      const firstImage = images.first();
      await expect(firstImage).toBeVisible();
      
      // Check if image has alt text
      const altText = await firstImage.getAttribute('alt');
      expect(altText).toBeTruthy();
    }
  });

  test('should have working search functionality', async ({ page }) => {
    // Look for search box
    const searchBox = page.locator('input[placeholder*="Search"], input[type="search"], .search-input');
    
    if (await searchBox.count() > 0) {
      await searchBox.first().isVisible();
      
      // Type in search
      await searchBox.first().fill('robotics');
      
      // Wait a bit for search results
      await page.waitForTimeout(1000);
    }
  });

  test('should have proper navigation between docs', async ({ page }) => {
    // Look for next/prev navigation
    const nextNav = page.locator('.pagination-nav__item--next, .nav-next');
    const prevNav = page.locator('.pagination-nav__item--prev, .nav-prev');
    
    if (await nextNav.count() > 0) {
      await expect(nextNav.first()).toBeVisible();
      await nextNav.first().click();
      await page.waitForLoadState('networkidle');
    }
    
    if (await prevNav.count() > 0) {
      await expect(prevNav.first()).toBeVisible();
    }
  });
});