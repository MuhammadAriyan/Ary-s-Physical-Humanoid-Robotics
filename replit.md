# Ary's Physical & Humanoid Robotics

## Overview
A comprehensive Docusaurus documentation website covering Physical AI and humanoid robotics, from fundamental concepts to advanced applications in embodied intelligence. Includes multilingual support (English and Urdu).

## Project Structure
- `docs/` - Main documentation content organized by parts
- `i18n/ur/` - Urdu translations
- `src/` - Custom React components and pages
- `static/` - Static assets (images, icons)
- `backend/` - Python FastAPI backend (for chat/RAG features - external service)
- `auth-service/` - Authentication service (external service)

## Development

### Running Locally
The Docusaurus dev server runs on port 5000:
```bash
npm run start -- --host 0.0.0.0 --port 5000
```

### Build for Production
```bash
npm run build
```

Output is generated in the `build/` directory.

## Configuration
- `docusaurus.config.ts` - Main Docusaurus configuration
- `sidebars.ts` - Sidebar navigation configuration

## Environment Variables
- `BASE_URL` - Base URL path (set to `/` for Replit)

## Recent Changes
- 2026-01-04: Configured for Replit environment with port 5000
