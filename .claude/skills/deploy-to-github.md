---
name: deploy
description: Run type check and build, then commit and push

---

Run `npm run typecheck` and `npm run build`, then create a commit and push.

**Steps:**
1. Run type check (`npm run typecheck`)
2. Run build (`npm run build`)
3. Stage and commit changes with concise message
4. Push to remote

**Parameters:**
- `message`: Optional custom commit message (defaults to auto-generated)

**Example:**
```
/deploy
/deploy "feat: add new feature"
```
