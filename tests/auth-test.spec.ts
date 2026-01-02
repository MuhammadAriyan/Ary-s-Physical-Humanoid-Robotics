import { test, expect } from '@playwright/test';

const SITE_URL = 'https://muhammadariyan.github.io/Ary-s-Physical-Humanoid-Robotics/chat';
const EMAIL = '2aryan4march@gmail.com';
const PASSWORD = '12121212';

test.describe('Authentication Flow - Production', () => {
  test.beforeEach(async ({ page, context }) => {
    // Enable console logging
    page.on('console', msg => console.log(`PAGE LOG [${msg.type()}]:`, msg.text()));
    page.on('pageerror', err => console.error('PAGE ERROR:', err));
    context.on('requestfailed', request => {
      console.log('REQUEST FAILED:', request.url(), request.failure());
    });
    context.on('response', async response => {
      console.log('RESPONSE:', response.url(), response.status());
    });

    // Navigate to chat page
    await page.goto(SITE_URL, { waitUntil: 'networkidle' });
  });

  test('should show auth modal for unauthenticated users', async ({ page }) => {
    console.log('='.repeat(50));
    console.log('TEST: Check auth modal visibility');

    // Wait for modal to appear
    const modal = page.locator('div[class*="modal"]').first();
    await expect(modal).toBeVisible({ timeout: 10000 }).catch(async (e) => {
      console.log('Modal not visible:', e);
      await page.screenshot({ path: 'test-results/auth-modal-not-visible.png' });
      throw e;
    });

    // Check for sign-in/sign-up title
    const title = page.locator('h2').first();
    const titleText = await title.textContent();
    console.log('Modal title:', titleText);

    // Take screenshot
    await page.screenshot({ path: 'test-results/auth-modal.png' });
  });

  test('should toggle between sign-in and sign-up', async ({ page }) => {
    console.log('='.repeat(50));
    console.log('TEST: Toggle sign-in/sign-up mode');

    // Ensure modal is visible
    const modal = page.locator('div[class*="modal"]').first();
    await expect(modal).toBeVisible();

    // Click "Sign Up" toggle button at bottom
    const signUpToggle = page.getByRole('button', { name: /Sign Up/i });
    await signUpToggle.click();

    // Wait for title change
    await page.waitForTimeout(500);

    // Check for "Create Account" title
    const title = page.locator('h2').first();
    const titleText = await title.textContent();
    expect(titleText).toMatch(/Create Account|Sign Up/i);
    console.log('After toggle, title:', titleText);

    await page.screenshot({ path: 'test-results/sign-up-mode.png' });
  });

  test('should handle sign-up with credentials', async ({ page }) => {
    console.log('='.repeat(50));
    console.log('TEST: Sign up with credentials');

    // Ensure modal is in sign-up mode
    let signUpToggle = page.getByRole('button', { name: /Sign Up/i });
    await signUpToggle.click().catch(() => {});
    await page.waitForTimeout(500);

    // Fill in credentials
    await page.fill('#email', EMAIL);
    await page.fill('#password', PASSWORD);
    await page.fill('#name', 'Test User');

    // Take screenshot before submit
    await page.screenshot({ path: 'test-results/before-submit.png' });

    console.log('Submitting form with:', EMAIL);

    // Submit form
    const submitButton = page.locator('button[type="submit"]');
    await submitButton.click();

    // Wait for response - check for modal to close or error
    await page.waitForTimeout(5000);

    // Check for any error messages
    const error = page.locator('div[class*="error"]').first();
    const hasError = await error.count() > 0;
    if (hasError) {
      const errorText = await error.textContent();
      console.log('ERROR:', errorText);
      await page.screenshot({ path: 'test-results/error-message.png' });
    } else {
      console.log('No error visible, checking if modal closed or authenticated');

      // Check if modal closed
      const modal = page.locator('div[class*="modal"]').first();
      const modalVisible = await modal.isVisible().catch(() => false);
      console.log('Modal still visible:', modalVisible);

      // Check if authenticated (sign out button appears)
      const signOutButton = page.getByTitle('Sign out');
      const signOutVisible = await signOutButton.isVisible().catch(() => false);
      console.log('Sign out button visible:', signOutVisible);

      await page.screenshot({ path: 'test-results/after-submit.png' });
    }
  });

  test('should show sign-up button in chat header', async ({ page }) => {
    console.log('='.repeat(50));
    console.log('TEST: Check sign-up button visibility');

    // Sign-up button should be visible when not authenticated
    const signUpButton = page.getByTitle('Sign up');
    const isVisible = await signUpButton.isVisible().catch(() => false);
    console.log('Sign-up button visible:', isVisible);

    if (!isVisible) {
      // User might be authenticated - check
      const chatTitle = page.locator('h1[class*="chatTitle"]');
      console.log('Chat title:', await chatTitle.textContent());
    }
  });

  test('should check localStorage values', async ({ page }) => {
    console.log('='.repeat(50));
    console.log('TEST: Check localStorage values');

    // Get all auth-related localStorage values
    const authUser = await page.evaluate(() => localStorage.getItem('fubuni_auth_user'));
    const authTimestamp = await page.evaluate(() => localStorage.getItem('fubuni_auth_timestamp'));
    const jwtToken = await page.evaluate(() => localStorage.getItem('fubuni_jwt_token'));

    console.log('localStorage values:');
    console.log('  fubuni_auth_user:', authUser);
    console.log('  fubuni_auth_timestamp:', authTimestamp);
    console.log('  fubuni_jwt_token:', jwtToken ? 'exists' : 'none');

    // If auth user exists, check timestamp age
    if (authUser && authTimestamp) {
      const age = Date.now() - parseInt(authTimestamp);
      const sevenDays = 7 * 24 * 60 * 60 * 1000;
      console.log('  Timestamp age (ms):', age);
      console.log('  Seven days (ms):', sevenDays);
      console.log('  Is expired:', age >= sevenDays);
    }

    await page.screenshot({ path: 'test-results/localstorage-check.png' });
  });

  test('should check for Better Auth session and API', async ({ page }) => {
    console.log('='.repeat(50));
    console.log('TEST: Check Better Auth session and API');

    // Check for session cookie
    const cookies = await page.context().cookies();
    console.log('Cookies:', cookies.map(c => ({
      name: c.name,
      domain: c.domain,
      secure: c.secure,
      sameSite: c.sameSite
    })));

    // Try to call session endpoint on Koyeb
    try {
      const response = await page.request.get('https://gorgeous-deanne-ary-s-88e09c71.koyeb.app/api/auth/session');
      console.log('Session API status:', response.status());
      console.log('Session API body:', await response.text());
    } catch (error) {
      console.log('Session API error:', error);
    }

    // Try to call health endpoint
    try {
      const healthResponse = await page.request.get('https://gorgeous-deanne-ary-s-88e09c71.koyeb.app/api/health');
      console.log('Health API status:', healthResponse.status());
      console.log('Health API body:', await healthResponse.text());
    } catch (error) {
      console.log('Health API error:', error);
    }

    await page.screenshot({ path: 'test-results/api-check.png' });
  });

  test('close modal on backdrop click', async ({ page }) => {
    console.log('='.repeat(50));
    console.log('TEST: Close modal on backdrop click');

    const modal = page.locator('div[class*="modal"]').first();
    const backdrop = page.locator('div[class*="backdrop"]').first();

    // Click backdrop
    await backdrop.click();

    // Modal should close
    await expect(modal).not.toBeVisible({ timeout: 5000 });
  });

  test('debug: collect all information', async ({ page, context }) => {
    console.log('='.repeat(50));
    console.log('DEBUG INFORMATION COLLECTION');
    console.log('='.repeat(50));

    await page.goto(SITE_URL, { waitUntil: 'networkidle' });

    // Page title and URL
    console.log('Page URL:', page.url());
    console.log('Page title:', await page.title());

    // Console messages (already set up in beforeEach)
    console.log('See above for console logs, errors, and network failures');

    // Check if AuthProvider loaded
    const authProviderLoaded = await page.evaluate(() => {
      return (window as any).__authProviderLoaded === true;
    });
    console.log('AuthProvider loaded:', authProviderLoaded);

    // Check useSession hook state
    const sessionState = await page.evaluate(() => {
      return (window as any).__sessionState;
    });
    console.log('Session state:', sessionState);

    // Network requests (already set up in beforeEach)
    console.log('See above for network request/response logs');

    await page.screenshot({ path: 'test-results/debug-overview.png' });
    console.log('Screenshot saved to: test-results/debug-overview.png');
    console.log('='.repeat(50));
  });
});
