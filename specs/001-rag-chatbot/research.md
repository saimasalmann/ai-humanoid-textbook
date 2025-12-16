# Research Summary: RAG-Enabled AI Textbook Chatbot

## Decision: AI Model Selection
**Rationale**: Need to create an agent compatible with OpenAI Agents SDK interface but using Google's Gemini model via Google API key.
**Chosen**: Custom agent implementation that follows OpenAI Agents SDK patterns but uses Google's Gemini model
**Alternatives considered**:
- Native OpenAI integration: Would require OpenAI API key instead of Google API key
- Native Google AI SDK: Would require different interface than OpenAI Agents SDK
- Third-party abstraction layer: May add unnecessary complexity

## Decision: Qdrant Integration Approach
**Rationale**: Need to connect to existing Qdrant Cloud collection with book embeddings
**Chosen**: Use qdrant-client Python library to connect to Qdrant Cloud
**Alternatives considered**:
- Direct HTTP API calls: More complex, less maintainable
- Pydantic models with qdrant-client: Provides type safety and easier integration

## Decision: Session Management Strategy
**Rationale**: Need to maintain chat history and session data with 30-day retention
**Chosen**: Neon Serverless Postgres with automatic cleanup using scheduled tasks
**Alternatives considered**:
- In-memory storage: Doesn't persist across deployments
- Redis: Additional infrastructure complexity
- Local SQLite: Not suitable for production deployment

## Decision: Rate Limiting Implementation
**Rationale**: Need to enforce 10 requests per minute per session
**Chosen**: Token bucket algorithm with Redis backend for distributed rate limiting
**Alternatives considered**:
- Simple counter: Doesn't handle time windows properly
- In-memory rate limiter: Doesn't work across multiple instances
- Database-based: Higher latency than Redis

## Decision: Frontend Integration Method
**Rationale**: Need to integrate chatbot UI into existing Docusaurus structure without changes
**Chosen**: React component integrated via Docusaurus swizzling or plugin
**Alternatives considered**:
- Iframe embedding: More isolated but less integrated
- Custom Docusaurus plugin: More maintainable but complex to develop
- Direct script injection: Simplest but less maintainable

## Decision: Text Selection Capture
**Rationale**: Need to capture user-selected text for selected-text mode
**Chosen**: JavaScript event listeners for text selection with context menu integration
**Alternatives considered**:
- Browser extension: Requires separate installation
- Right-click context menu: More intuitive for users
- Toolbar button: Always visible but potentially cluttered

## Decision: Error Handling Strategy
**Rationale**: Need graceful degradation when external services fail
**Chosen**: Circuit breaker pattern with fallback messages
**Alternatives considered**:
- Retry with exponential backoff: Good for temporary failures
- Immediate failure: Provides quick feedback but poor UX
- Silent degradation: Hides problems from users