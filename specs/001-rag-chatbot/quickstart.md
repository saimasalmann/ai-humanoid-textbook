# Quickstart Guide: RAG-Enabled AI Textbook Chatbot

## Development Setup

### Prerequisites
- Python 3.11+
- Node.js 20+ (for Docusaurus frontend)
- Access to Qdrant Cloud instance with textbook embeddings
- Neon Serverless Postgres database
- Google API key

### Backend Setup

1. **Create the backend directory structure:**
```bash
mkdir -p backend/{src,tests}
mkdir -p backend/src/{models,services,api,config,utils}
```

2. **Install Python dependencies:**
```bash
cd backend
pip install fastapi uvicorn python-dotenv openai qdrant-client asyncpg aiopg psycopg2-binary redis
```

3. **Create requirements.txt:**
```txt
fastapi==0.104.1
uvicorn==0.24.0
python-dotenv==1.0.0
openai==1.3.7
qdrant-client==1.7.0
asyncpg==0.29.0
redis==5.0.1
pydantic==2.5.0
pytest==7.4.3
```

4. **Set up environment variables (.env):**
```env
GOOGLE_API_KEY=your_google_api_key
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
NEON_DATABASE_URL=your_neon_database_url
REDIS_URL=redis://localhost:6379
RATE_LIMIT_REQUESTS=10
RATE_LIMIT_WINDOW=60
SESSION_RETENTION_DAYS=30
```

### Frontend Integration

1. **Install ChatKit.js in the Docusaurus project:**
```bash
npm install @pusher/chatkit-client
```

2. **Create a chat widget component that can be embedded in Docusaurus pages**

3. **Implement text selection capture functionality**

## Running the Application

### Backend
```bash
cd backend
uvicorn src.api.main:app --reload --port 8000
```

### Frontend
```bash
cd (docusaurus-project-root)
npm start
```

## API Usage

### Chat Endpoint
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "sessionId": "test-session-123",
    "message": "What is forward kinematics?",
    "queryMode": "full-book"
  }'
```

### Health Check
```bash
curl http://localhost:8000/health
```

## Key Components

### 1. Qdrant Service (`src/services/qdrant_service.py`)
Handles vector search and retrieval from Qdrant Cloud

### 2. RAG Agent (`src/services/rag_agent.py`)
Implements the OpenAI agent with RAG capabilities

### 3. Chat Service (`src/services/chat_service.py`)
Manages chat sessions and message history

### 4. Rate Limiter (`src/services/rate_limiter.py`)
Implements rate limiting per session

## Testing

### Unit Tests
```bash
cd backend
pytest tests/unit/
```

### Integration Tests
```bash
cd backend
pytest tests/integration/
```

## Error Checking and Optimization

### Frontend Error Handling
1. **Error Boundaries**: The ChatWidget is wrapped in error boundaries to catch JavaScript errors
2. **API Error Handling**: Network errors show user-friendly messages in the chat interface
3. **Input Validation**: Messages are validated before sending to prevent malformed requests
4. **Accessibility**: All error messages are accessible to screen readers

### Backend Error Handling
1. **Pydantic Validation**: All inputs validated with Pydantic models at API endpoints
2. **Custom Exception Handlers**: Specific handlers for different error types (validation, business logic, infrastructure)
3. **Rate Limiting**: Enforced at 10 requests/minute per session with proper error responses
4. **Service Failures**: Circuit breaker pattern for external services (Qdrant, Gemini API)
5. **Structured Logging**: Comprehensive logging with correlation IDs for debugging

### Image Optimization
- All current images are SVG format (technical diagrams) optimized with SVGO
- Located in `frontend/static/img/` directory
- Total size ~33KB with potential for additional 15KB savings through enhanced optimization
- For new images: Use SVG for diagrams, WebP for photos with fallbacks

### Validation Scripts
Run before deployment to ensure quality:
```bash
# Backend validation
cd backend
python -m pytest tests/  # Run all tests
python -c "import src.services.chat_service; print('Chat service imports successfully')"  # Import validation

# Frontend validation
cd frontend
npm run build  # Build validation
```

## Testing

### Unit Tests
```bash
cd backend
pytest tests/unit/
```

### Integration Tests
```bash
cd backend
pytest tests/integration/
```

### Error Path Tests
Test error handling scenarios:
```bash
# Test rate limiting
for i in {1..15}; do
  curl -X POST http://localhost:8000/chat -H "Content-Type: application/json" \
    -d '{"sessionId": "test-rate-limit", "message": "Test message '"$i"'", "queryMode": "full-book"}'
  sleep 0.5
done

# Test invalid inputs
curl -X POST http://localhost:8000/chat -H "Content-Type: application/json" \
  -d '{"sessionId": "", "message": "", "queryMode": "invalid-mode"}'
```

## Deployment

The backend can be deployed independently on platforms like Render, Railway, or any cloud provider that supports Python/FastAPI applications.

### Pre-deployment Checklist
- [ ] All tests pass (unit, integration, error paths)
- [ ] Error boundaries are functional
- [ ] Rate limiting is enforced and tested
- [ ] Images are optimized and validated
- [ ] Logging is configured for production
- [ ] Health check endpoint returns healthy status
- [ ] Security headers are properly set
- [ ] Performance metrics meet requirements