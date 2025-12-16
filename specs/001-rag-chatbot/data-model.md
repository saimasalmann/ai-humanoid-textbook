# Data Model: RAG-Enabled AI Textbook Chatbot

## Entities

### ChatSession
- **sessionId** (string): Unique identifier for the chat session
- **userId** (string, optional): Anonymous user identifier (for tracking purposes)
- **createdAt** (datetime): Timestamp when session was created
- **lastActiveAt** (datetime): Timestamp of last activity
- **expiresAt** (datetime): Expiration timestamp (30 days from last activity)

### Message
- **messageId** (string): Unique identifier for the message
- **sessionId** (string): Reference to parent ChatSession
- **role** (string): Message role (user, assistant, system)
- **content** (string): Message content
- **timestamp** (datetime): When the message was created
- **queryMode** (string): Query mode ('full-book' or 'selected-text')
- **selectedText** (string, optional): Text selected by user (if in selected-text mode)

### QueryRequest
- **requestId** (string): Unique identifier for the request
- **sessionId** (string): Reference to associated ChatSession
- **userQuery** (string): Original user question
- **selectedText** (string, optional): Text selected by user (if in selected-text mode)
- **queryMode** (string): Query mode ('full-book' or 'selected-text')
- **timestamp** (datetime): When the request was made
- **metadata** (json): Additional metadata (chapter, section, page if available)

### RetrievedContext
- **contextId** (string): Unique identifier for the retrieved context
- **requestId** (string): Reference to associated QueryRequest
- **content** (string): Retrieved text content from Qdrant
- **metadata** (json): Metadata from Qdrant (chapter, section, page, etc.)
- **similarityScore** (float): Similarity score from vector search
- **rank** (integer): Rank in the retrieval results

## Relationships

- ChatSession (1) → (Many) Message: A chat session contains multiple messages
- ChatSession (1) → (Many) QueryRequest: A chat session contains multiple query requests
- QueryRequest (1) → (Many) RetrievedContext: A query request may retrieve multiple context chunks

## Validation Rules

### ChatSession
- sessionId must be unique
- createdAt must be before lastActiveAt
- expiresAt must be 30 days after lastActiveAt
- userId is optional but if present, must follow anonymous user format

### Message
- role must be one of: 'user', 'assistant', 'system'
- content must not exceed 10,000 characters
- queryMode must be one of: 'full-book', 'selected-text'
- if queryMode is 'selected-text', selectedText must not be null

### QueryRequest
- queryMode must be one of: 'full-book', 'selected-text'
- userQuery must not be empty
- if queryMode is 'selected-text', selectedText must not be null

### RetrievedContext
- similarityScore must be between 0 and 1
- rank must be a positive integer

## Backend Error Handling Requirements

### 1. Input Validation Errors
- **Pydantic Validation**: All request bodies must be validated against Pydantic models
- **Type Conversion Errors**: Handle invalid data types gracefully
- **Missing Required Fields**: Return appropriate HTTP 400 errors
- **Field Validation**: Validate field constraints (regex, min/max values, etc.)

### 2. Business Logic Errors
- **Session Validation**: Check if sessions exist and are not expired
- **Query Mode Validation**: Validate that query modes are supported
- **Selected Text Validation**: Ensure selected text is provided when needed
- **Rate Limiting**: Enforce 10 requests per minute per session

### 3. Infrastructure Errors
- **Database Connection**: Handle connection timeouts and pool exhaustion
- **Qdrant Service**: Handle vector database failures gracefully
- **AI Service**: Handle Gemini API failures with fallback responses
- **Resource Limits**: Monitor and handle memory/CPU exhaustion

### 4. Authentication and Authorization
- **Session Management**: Validate session tokens and permissions
- **Rate Limiting**: Enforce per-session request limits
- **Request Validation**: Ensure requests come from valid sessions

## Error Response Format

All errors should follow this structure:

```json
{
  "error": {
    "type": "ValidationError|BusinessException|InfrastructureError|AuthenticationError",
    "message": "Human-readable error message",
    "details": "Additional error details",
    "timestamp": "ISO 8601 timestamp",
    "requestId": "Unique request identifier"
  }
}
```

## Error Recovery Strategies

### 1. Circuit Breaker Pattern
- Track service failures and temporarily disable failing services
- Implement fallback responses when external services are unavailable
- Gradually restore services after recovery period

### 2. Retry Mechanisms
- Implement exponential backoff for transient failures
- Limit retry attempts to prevent resource exhaustion
- Use different strategies for different error types

### 3. Graceful Degradation
- Provide basic functionality when advanced features fail
- Maintain core functionality during partial service outages
- Inform users of reduced functionality with clear messages

## Backend Validation Checklist

### Before Production:
- [ ] All Pydantic models validate input correctly
- [ ] Error handlers catch and format all exceptions
- [ ] Database connection pooling is properly configured
- [ ] Rate limiting is enforced per session
- [ ] Session expiration is handled correctly
- [ ] External service failures are handled gracefully
- [ ] Logging includes correlation IDs for debugging
- [ ] Health check endpoints are available and functional
- [ ] Performance metrics are monitored and alertable
- [ ] Security headers are properly set

### Testing Requirements:
- [ ] Unit tests cover all validation scenarios
- [ ] Integration tests verify error handling paths
- [ ] Load tests verify rate limiting works correctly
- [ ] Error recovery scenarios are tested
- [ ] Database connection handling under stress is verified
- [ ] Field name consistency between models and usage is verified (e.g., session_id vs sessionId)