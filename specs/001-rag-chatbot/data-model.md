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