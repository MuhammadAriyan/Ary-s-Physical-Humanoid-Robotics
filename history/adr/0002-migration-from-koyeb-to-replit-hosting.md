# ADR-0002: Migration from Koyeb to Replit Hosting

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2026-01-03
- **Feature:** 012-migrate-auth-replit
- **Context:** Migrate the Better Auth + Express authentication service from Koyeb hosting to Replit hosting platform due to Koyeb free trial ending. This decision impacts the operational architecture, deployment strategy, and service availability characteristics. The migration requires adapting the service configuration to work with Replit's runtime environment, including port binding, CORS configuration, and environment variable management.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

- **Hosting Platform**: Replit Node.js environment for auth service deployment
- **Server Configuration**: Listen on Replit's PORT environment variable and bind to 0.0.0.0 for external access
- **CORS Configuration**: Update allowed origins to include Replit URL patterns
- **Environment Management**: Use Replit's secrets system for sensitive configuration values
- **Service Startup**: Adapt Express/Better Auth startup to Replit's execution model
- **URL Configuration**: Update BETTER_AUTH_URL and related configuration to Replit domain patterns
- **Sleep/Wake Handling**: Accept Replit free tier sleep behavior with potential cold starts

<!-- For technology stacks, list all components:
     - Framework: Next.js 14 (App Router)
     - Styling: Tailwind CSS v3
     - Deployment: Vercel
     - State Management: React Context (start simple)
-->

## Consequences

### Positive

- Zero operational cost for hosting during development phase
- Quick deployment and iteration capabilities with Replit's integrated development environment
- No need to manage separate infrastructure for development/auth service
- Familiar Node.js runtime environment compatible with existing auth service
- Easy collaboration and sharing capabilities if needed in the future
- No credit card required for basic service hosting

<!-- Example: Integrated tooling, excellent DX, fast deploys, strong TypeScript support -->

### Negative

- Service may experience cold starts due to Replit's free tier sleep behavior
- Potential downtime when service goes to sleep and needs to wake up for requests
- Limited resource allocation compared to paid hosting platforms
- Dependency on Replit's platform stability and policies
- Not suitable for production use due to free tier limitations
- Possible rate limiting on the Replit free tier

## Alternatives Considered

**Alternative A**: Continue with Koyeb hosting using paid subscription
- Why rejected: Cost consideration - Koyeb free trial ended and ongoing costs were not justified for development phase

**Alternative B**: Use Railway for hosting
- Why rejected: Also has limited free hours and requires account verification, with similar cost implications to Koyeb

**Alternative C**: Use Render for hosting
- Why rejected: Free tier has sleep cycles similar to Replit, but with more complex deployment setup

**Alternative D**: Self-host with ngrok
- Why rejected: Requires keeping a computer running continuously, not suitable for reliable service availability

**Alternative E**: Use Fly.io for hosting
- Why rejected: Requires more complex configuration and account setup, with potential costs beyond free tier limits

<!-- Group alternatives by cluster:
     Alternative Stack A: Remix + styled-components + Cloudflare
     Alternative Stack B: Vite + vanilla CSS + AWS Amplify
     Why rejected: Less integrated, more setup complexity
-->

## References

- Feature Spec: /specs/012-migrate-auth-replit/spec.md
- Implementation Plan: /specs/012-migrate-auth-replit/plan.md
- Related ADRs: ADR-0001 (auth-service-consolidation-approach.md)
- Evaluator Evidence: /history/prompts/012-migrate-auth-replit/ (PHR records)
