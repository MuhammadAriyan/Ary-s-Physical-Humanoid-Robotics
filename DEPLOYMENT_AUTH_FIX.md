# Cross-Domain Authentication Fix - Deployment Guide

## Problem Fixed

The authentication modal was appearing after successful sign-up/sign-in because session cookies from the Koyeb auth service (`ary-s-physical-humanoid-robotics--maryanrar.replit.app`) could not be read by the GitHub Pages frontend (`muhammadariyan.github.io`).

## Solution Implemented

**Token-based authentication with localStorage persistence** combined with cross-origin cookie configuration.

### Key Changes

1. **Auth Service Cookie Configuration** - Set cookies with `SameSite=None; Secure` for cross-origin
2. **Frontend Auth Client** - Added `credentials: 'include'` to send cookies cross-origin
3. **Token Persistence** - Store user data in localStorage immediately after authentication
4. **Auth State Detection** - Check localStorage first (reliable across domains)

## Deployment Instructions

### 1. Update Koyeb Environment Variables

In your Koyeb auth service deployment, set these environment variables:

```env
NODE_ENV=production
BETTER_AUTH_URL=https://ary-s-physical-humanoid-robotics--maryanrar.replit.app
CORS_ORIGINS=https://muhammadariyan.github.io
BETTER_AUTH_SECRET=<your-32-char-secret>
JWT_SECRET=<your-jwt-secret>
DATABASE_URL=<your-neon-postgres-url>
GOOGLE_CLIENT_ID=<your-google-client-id>
GOOGLE_CLIENT_SECRET=<your-google-client-secret>
```

**Important**:
- `BETTER_AUTH_URL` must be your actual Koyeb deployment URL (no trailing slash)
- `CORS_ORIGINS` must be exact GitHub Pages URL (no trailing slash)
- If URL changes, update both env variables

### 2. Verify Google OAuth Configuration

In Google Cloud Console (https://console.cloud.google.com/apis/credentials):

**Authorized redirect URIs:**
```
https://ary-s-physical-humanoid-robotics--maryanrar.replit.app/api/auth/callback/google
```

**Authorized JavaScript origins:**
```
https://muhammadariyan.github.io
https://gorgeous-deeanne-ary-s-88e09c71.koyeb.app
```

### 3. Deploy Auth Service to Koyeb

```bash
# From project root
cd auth-service
git add .
git commit -m "fix: configure cross-origin authentication"
git push
```

Koyeb will automatically deploy the updated auth service.

### 4. Deploy Frontend to GitHub Pages

```bash
# From project root
npm run build
# Commit the build output if needed
git add .
git commit -m "fix: implement token-based cross-domain auth"
git push
```

GitHub Actions will automatically deploy to GitHub Pages.

### 5. Verify Deployment

#### Test CORS Configuration

```bash
curl -i -X OPTIONS https://gorgeous-deeanne-ary-s-88e09c71.koyeb.app/api/auth/session \
  -H "Origin: https://muhammadariyan.github.io" \
  -H "Access-Control-Request-Method: GET"
```

**Expected response headers:**
```
access-control-allow-origin: https://muhammadariyan.github.io
access-control-allow-credentials: true
```

#### Test in Browser

1. Open https://muhammadariyan.github.io/Ary-s-Physical-Humanoid-Robotics/chat
2. Sign up with email/password or Google
3. **Expected**: Modal closes immediately after successful authentication
4. Refresh page
5. **Expected**: You remain authenticated (no modal appears)
6. Open DevTools → Application → Local Storage
7. **Expected**: See `fubuni_auth_user` and `fubuni_auth_timestamp`
8. Open DevTools → Application → Cookies → `gorgeous-deeanne-ary-s-88e09c71.koyeb.app`
9. **Expected**: See `better-auth.session_token` with `SameSite=None` and `Secure` flags

## Files Modified

### Backend (auth-service/)

1. **src/auth.ts** (lines 66-82)
   - Added GitHub Pages to `trustedOrigins`
   - Configured `defaultCookieAttributes` with `sameSite: 'none'` for production

2. **src/index.ts** (lines 15-24)
   - Added GitHub Pages to CORS origins

### Frontend (src/)

3. **lib/auth-client.ts** (lines 15-20)
   - Added `fetchOptions: { credentials: 'include' }`

4. **components/Auth/AuthModal.tsx** (lines 39-50, 66-77)
   - Store session in localStorage after sign-up/sign-in

5. **pages/chat.tsx** (lines 161-174)
   - Check localStorage first for auth state
   - Increased delay to 2 seconds for cross-origin propagation
   - Removed unreliable cookie check

## How It Works

### Authentication Flow

1. **User Signs Up/In**:
   - Frontend calls auth service API
   - Auth service creates session and sets cookie with `SameSite=None; Secure`
   - Cookie is sent to `koyeb.app` domain

2. **Token Persistence**:
   - Frontend immediately calls `getSession()` after authentication
   - User data is stored in localStorage as `fubuni_auth_user`
   - Timestamp is stored as `fubuni_auth_timestamp`

3. **Auth State Detection**:
   - On page load, check localStorage FIRST (reliable across domains)
   - If localStorage has auth data, assume authenticated
   - If Better Auth session exists, use that as secondary check
   - Only show auth modal if BOTH checks fail

4. **API Requests**:
   - All API calls include `credentials: 'include'`
   - Cookies are sent cross-origin (if browser allows)
   - JWT tokens are used for backend authentication

### Cross-Origin Cookie Behavior

**What works:**
- Cookies are set on `koyeb.app` domain with `SameSite=None; Secure`
- Cookies are sent with API requests from `github.io` to `koyeb.app`
- localStorage works reliably across page refreshes

**What doesn't work:**
- JavaScript on `github.io` cannot read cookies from `koyeb.app` (browser security)
- Some browsers may block third-party cookies entirely

**Our solution:**
- Use localStorage as primary auth indicator (reliable)
- Use Better Auth session state as secondary check
- Cookies still work for API authentication

## Troubleshooting

### Issue: Modal still appears after sign-in

**Check:**
1. Open DevTools → Console → Look for CORS errors
2. Check if localStorage has `fubuni_auth_user`
3. Check if cookies have `SameSite=None` and `Secure` flags

**Solutions:**
- Clear browser cache and cookies
- Try in incognito mode
- Verify `NODE_ENV=production` on Koyeb
- Verify CORS_ORIGINS has exact URL (no trailing slash)

### Issue: CORS errors in console

**Check:**
1. Verify `CORS_ORIGINS` environment variable on Koyeb
2. Check that URL matches exactly (no trailing slash)

**Solutions:**
- Update Koyeb environment variables
- Redeploy auth service
- Wait 2-3 minutes for deployment to complete

### Issue: Cookies not being set

**Check:**
1. Open DevTools → Application → Cookies
2. Look for `better-auth.session_token` on `koyeb.app` domain
3. Check if `SameSite` and `Secure` attributes are set

**Solutions:**
- Verify `NODE_ENV=production` on Koyeb
- Verify auth service is running on HTTPS (not HTTP)
- Check Better Auth configuration in `auth.ts`

### Issue: Authentication works but loses session on refresh

**Check:**
1. Check if localStorage persists across page loads
2. Verify cookies have proper `maxAge` or `expires` attributes

**Solutions:**
- Check session expiration settings in `auth.ts` (line 33-38)
- Ensure `fubuni_auth_user` is in localStorage
- Verify Better Auth session is not expiring immediately

## Testing Checklist

After deployment, test the following:

- [ ] Sign up with email/password → modal closes immediately
- [ ] Refresh page → stay authenticated (no modal)
- [ ] Sign out → modal appears
- [ ] Sign in with existing account → modal closes immediately
- [ ] Sign in with Google OAuth → modal closes after redirect
- [ ] Test in Chrome, Firefox, Safari (if available)
- [ ] Test with third-party cookies blocked (Privacy Badger, Brave)
- [ ] Verify localStorage has `fubuni_auth_user` after sign-in
- [ ] Verify cookies have `SameSite=None` and `Secure` flags

## Alternative Solutions (If Issues Persist)

### Option 1: Custom Domain (Recommended for Production)

Deploy both services under the same base domain:
- Frontend: `app.fubuni.com` or `fubuni.com`
- Auth Service: `api.fubuni.com` or `auth.fubuni.com`

This eliminates all cross-domain issues.

**Benefits:**
- No cross-domain cookie issues
- Better browser compatibility
- More professional URLs

**Setup:**
1. Purchase domain (e.g., fubuni.com)
2. Configure DNS:
   - `fubuni.com` → GitHub Pages
   - `api.fubuni.com` → Koyeb deployment
3. Update auth client URL and environment variables

### Option 2: Move Frontend to Same Platform

Deploy frontend to Koyeb alongside auth service:
- Frontend: `https://your-app.koyeb.app`
- Auth Service: `https://your-app.koyeb.app/api/auth`

**Benefits:**
- Same domain, no cross-origin issues
- Simpler deployment pipeline

## Support

If you encounter issues after following this guide:

1. Check browser console for errors
2. Verify all environment variables are set correctly
3. Test CORS configuration with curl command
4. Check Koyeb logs for auth service errors

## Additional Resources

- [Better Auth Documentation](https://www.better-auth.com/docs)
- [Better Auth Security Guide](https://www.better-auth.com/docs/reference/security)
- [MDN: SameSite Cookies](https://developer.mozilla.org/en-US/docs/Web/HTTP/Headers/Set-Cookie/SameSite)
- [Koyeb Documentation](https://www.koyeb.com/docs)
