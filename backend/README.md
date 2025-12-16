# RAG Chatbot Backend

This backend service provides a RAG (Retrieval-Augmented Generation) chatbot for the AI textbook, allowing students to ask questions about textbook content with responses grounded strictly in book content.

## Features

- RAG-based question answering using textbook content
- Support for both full-book and selected-text query modes
- Session management with 30-day retention
- Rate limiting (10 requests per minute per session)
- Graceful degradation when external services are unavailable

## Tech Stack

- FastAPI: Web framework
- Google Gemini: AI model for responses
- Qdrant: Vector database for textbook embeddings
- Neon Postgres: Session storage
- Redis: Rate limiting

## Setup

1. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. Ensure environment variables are set in `.env` file (in project root):
   - `GOOGLE_API_KEY`: Google API key for Gemini
   - `QDRANT_URL`: Qdrant Cloud URL
   - `QDRANT_API_KEY`: Qdrant API key
   - `NEON_DATABASE_URL`: Neon Postgres connection string (optional)
   - `REDIS_URL`: Redis connection URL

3. Start the development server:
   ```bash
   uvicorn src.api.main:app --reload --port 8000
   ```

## API Endpoints

- `POST /chat`: Process user queries and return AI responses
- `GET /health`: Check service health and dependencies

## Environment Variables

- `GOOGLE_API_KEY`: Google API key for accessing Gemini
- `QDRANT_URL`: URL for Qdrant vector database
- `QDRANT_API_KEY`: API key for Qdrant access
- `NEON_DATABASE_URL`: Database connection string for session storage
- `REDIS_URL`: Redis URL for rate limiting
- `RATE_LIMIT_REQUESTS`: Number of requests allowed per time window (default: 10)
- `RATE_LIMIT_WINDOW`: Time window in seconds (default: 60)
- `SESSION_RETENTION_DAYS`: Days to retain session data (default: 30)