---
id: 001
title: GitHub Pages API Implementation Blocked
stage: tasks
date: 2026-01-02
surface: agent
model: claude-opus-4.5-20251101
feature: 015-nextjs-github-pages-api
branch: main
user: ary
command: /sp.implement
labels: [implementation, blocked, architecture]
links:
  spec: specs/015-nextjs-github-pages-api/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/015-nextjs-github-pages-api/tasks.md
  - specs/015-nextjs-github-pages-api/plan.md
  - specs/015-nextjs-github-pages-api/spec.md
  - next-api/package.json
  - next-api/lib/auth.ts
  - next-api/lib/prisma.ts
  - next-api/prisma/schema.prisma
  - next-api/tsconfig.json
  - next-api/next.config.mjs
  - next-api/app/layout.tsx
  - next-api/app/api/auth/[...all]/route.ts
  - next-api/app/api/auth/token/route.ts
  - next-api/app/api/health/route.ts
  - src/lib/auth-client.ts
  - .github/workflows/deploy.yml
tests: []
---

## Prompt

/sp.implement

## Response snapshot

During implementation of feature "015-nextjs-github-pages-api", discovered that GitHub Pages is a static hosting service that does NOT support server-side API routes. The `output: 'export'` mode in Next.js explicitly removes all API routes during static build because they require server-side execution.

Completed most of the setup tasks:
- Created next-api directory structure
- Initialized Next.js project with TypeScript
- Installed dependencies (better-auth, prisma, jsonwebtoken)
- Configured TypeScript, Prisma, and Next.js configs
- Created Prisma schema and Better Auth instance
- Created auth route handlers, JWT token endpoint, health check endpoint
- Fixed auth-client.ts for same-origin API calls
- Added logging to auth routes
- Updated GitHub Actions workflow for parallel builds

The critical blocker was discovered when attempting to build: GitHub Pages cannot host server-side API routes.

## Outcome

- ✅ Impact: ABORTED - Reverted changes to original Koyeb setup
- 🧪 Tests: N/A
- 📁 Files: All changes reverted
- 🔁 Next prompts: Focus on fixing existing Koyeb CORS issues instead
- 🧠 Reflection: After researching RoboLearn project, confirmed that the correct architecture is: Docusaurus (GitHub Pages) + Backend API (separate server like Cloud Run/Koyeb). The spec was based on incorrect assumption that GitHub Pages could host API routes.

## Evaluation notes (flywheel)

- Failure modes observed: GitHub Pages static export removes API routes
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Rewrite spec to target Vercel/Netlify instead of GitHub Pages
