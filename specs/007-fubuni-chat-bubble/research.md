# Research: Fubuni Chat Bubble Implementation

## Decision: Use Custom Agents Library
**Rationale**: Based on user input, the implementation will use the custom agents library (imported as `from agents import Agent,Runner,AsyncOpenAI,OpenAIChatCompletionsModel,function_tool,ModelSettings`) rather than the official OpenAI Agents SDK. This approach provides better control and integration with custom providers like OpenRouter.

**Alternatives considered**:
- Official OpenAI Agents SDK: Rejected as user specifically requested the custom agents library
- LangChain: Would add unnecessary complexity for this use case

## Decision: OpenRouter as Primary Provider
**Rationale**: User specified using OpenRouter API with the endpoint `https://openrouter.ai/api/v1/chat/completions`. This allows access to various models with good performance and competitive pricing.

**Alternatives considered**:
- OpenAI API: Would require different implementation approach
- Self-hosted models: More complex setup and maintenance

## Decision: Streaming Implementation with SSE
**Rationale**: Requirement for token-by-token streaming responses can be achieved using Server-Sent Events (SSE) from FastAPI to provide real-time response streaming to the frontend.

**Alternatives considered**:
- WebSocket connection: More complex than needed for this use case
- Regular HTTP with full response: Doesn't meet streaming requirement

## Decision: Docusaurus Theme Integration
**Rationale**: To match the Docusaurus theme perfectly, we'll use Infima CSS classes and Docusaurus' swizzling capability to inject the chat component without modifying core files.

**Alternatives considered**:
- Custom CSS: Would not integrate as seamlessly with existing theme
- External styling: Would not respect dark/light mode automatically

## Decision: Neon Serverless Postgres for Data Storage
**Rationale**: Using Neon Serverless Postgres provides serverless, auto-scaling database capabilities that are cost-effective and require minimal management overhead. It provides PostgreSQL compatibility with branch-based development features.

**Implementation approach**:
- Use asyncpg for PostgreSQL connectivity in Python
- Implement connection pooling for optimal performance
- Use SQLModel or SQLAlchemy for ORM operations
- Store chat sessions and messages for conversation persistence

## Decision: Static-Friendly Architecture
**Rationale**: To support static Docusaurus builds, the frontend will make API calls to a separate backend service. The backend will connect to Neon Postgres for data persistence. For truly static deployments, we can implement a proxy or fallback mechanism.

**Implementation approach**:
- Frontend makes API calls to backend service
- Backend connects to Neon Postgres for data persistence
- For static deployment, use a proxy solution or static JSON fallback