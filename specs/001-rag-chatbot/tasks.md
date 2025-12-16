# Implementation Tasks: RAG-Enabled AI Textbook Chatbot

**Feature**: RAG-Enabled AI Textbook Chatbot
**Branch**: `001-rag-chatbot`
**Created**: 2025-12-14
**Input**: Feature specification from `/specs/001-rag-chatbot/spec.md`

## Implementation Strategy

The implementation will follow a phased approach starting with core infrastructure and progressing through user stories in priority order. The MVP will focus on User Story 1 (general questions about book content) with basic functionality before expanding to other user stories. Each phase builds upon the previous to ensure continuous integration and testing capability.

## Dependencies

- User Story 2 and User Story 3 depend on foundational backend components created in Phase 2
- User Story 2 requires Qdrant integration from User Story 1
- User Story 3 requires database persistence from User Story 1
- Frontend integration requires backend API completion

## Parallel Execution Examples

- Backend API development can run parallel to frontend component development
- Database schema design can run parallel to service layer implementation
- AI agent development can run parallel to data model implementation

---

## Phase 1: Setup

**Goal**: Initialize project structure and configure development environment

- [X] T001 Create backend directory structure per implementation plan: `backend/{src,tests}`, `backend/src/{models,services,api,config,utils}`
- [X] T002 create pyproject.toml file with dependencies: FastAPI, OpenAI Agents SDK, ChatKit Python, Qdrant client, asyncpg, redis, pydantic, pytest
- [X] T003 [P] create backend/.env with environment variables: GOOGLE_API_KEY, QDRANT_URL, QDRANT_API_KEY, NEON_DATABASE_URL, REDIS_URL, RATE_LIMIT_REQUESTS, RATE_LIMIT_WINDOW, SESSION_RETENTION_DAYS
- [X] T004 Create basic configuration module in `backend/src/config/settings.py` to handle environment variables
- [X] T005 Create project documentation files: README.md with setup instructions

---

## Phase 2: Foundational Components

**Goal**: Implement core infrastructure components that support all user stories

- [X] T006 [P] Create ChatSession model in `backend/src/models/chat_session.py` with sessionId, userId, createdAt, lastActiveAt, expiresAt fields
- [X] T007 [P] Create Message model in `backend/src/models/message.py` with messageId, sessionId, role, content, timestamp, queryMode, selectedText fields
- [X] T008 [P] Create QueryRequest model in `backend/src/models/query_request.py` with requestId, sessionId, userQuery, selectedText, queryMode, timestamp, metadata fields
- [X] T009 [P] Create RetrievedContext model in `backend/src/models/retrieved_context.py` with contextId, requestId, content, metadata, similarityScore, rank fields
- [X] T010 Create database connection module in `backend/src/services/database.py` for Neon Postgres
- [X] T011 Create Qdrant connection module in `backend/src/services/qdrant_client.py` for vector search
- [X] T012 Create rate limiter service in `backend/src/services/rate_limiter.py` using Redis with token bucket algorithm
- [X] T013 Create health check router in `backend/src/api/health_router.py` with dependency status checks
- [X] T014 Create main FastAPI application in `backend/src/api/main.py` with proper routing
- [X] T015 [P] Create utility functions in `backend/src/utils/validators.py` for input validation

---

## Phase 3: User Story 1 - Ask General Questions About Book Content (Priority: P1)

**Goal**: Implement core RAG functionality allowing students to ask questions about book content with AI responses grounded in textbook content

**Independent Test**: Can be fully tested by asking various questions about book content and verifying that the AI responds with accurate information from the textbook. Delivers immediate value by allowing students to get answers without searching through pages.

- [X] T016 [US1] Create Qdrant service in `backend/src/services/qdrant_service.py` to retrieve relevant content from Qdrant based on user queries
- [X] T017 [US1] Create RAG agent implementation in `backend/src/services/rag_agent.py` using OpenAI Agents SDK patterns with Google's Gemini model via AsyncOpenAI and OpenAIChatCompletionsModel with Google API key, including a dedicated tool for retrieving data from Qdrant
- [X] T018 [US1] Implement query validation in rag_agent.py to ensure responses are grounded only in retrieved book content with no hallucinations
- [X] T019 [US1] Create chat service in `backend/src/services/chat_service.py` to manage chat sessions and message flow
- [X] T020 [US1] Create chat router in `backend/src/api/chat_router.py` with POST /chat endpoint that accepts user message and session identifier
- [X] T021 [US1] Implement the POST /chat endpoint to process queries in full-book RAG mode (search entire book)
- [X] T022 [US1] Implement response formatting to display retrieved content separately from AI-generated responses
- [X] T023 [US1] Add rate limiting to chat endpoint using the rate limiter service (10 requests per minute per session)
- [X] T024 [US1] Create basic frontend chat widget component that can be embedded in Docusaurus pages
- [X] T025 [US1] Integrate chat interface into the Docusaurus frontend for basic chat functionality
- [X] T026 [US1] Implement frontend to backend API communication for sending questions and receiving responses
- [X] T027 [US1] Add graceful degradation handling when external services (Qdrant/Postgres) are unavailable
- [X] T028 [US1] Create session management to maintain chat history and store conversations in Neon Postgres with automatic deletion after 30 days

---

## Phase 4: User Story 2 - Ask Questions About Selected Text (Priority: P2)

**Goal**: Enhance the chat functionality to allow students to select specific text on a page and ask questions only about that selected text

**Independent Test**: Can be tested by selecting text on a page, asking a question about it, and verifying that the AI response is limited to the selected text context rather than drawing from the entire book.

- [X] T029 [US2] Enhance RAG agent in `backend/src/services/rag_agent.py` to support selected-text mode for answers generated only from user-selected text
- [X] T030 [US2] Update chat service to handle selected text context when provided in the query
- [X] T031 [US2] Modify POST /chat endpoint to accept optional selectedText parameter and route to selected-text mode
- [X] T032 [US2] Implement query mode selection in the backend to distinguish between full-book and selected-text queries
- [X] T033 [US2] Create JavaScript functionality to capture user-selected text from textbook pages
- [X] T034 [US2] Add text selection UI in the frontend chat widget with appropriate styling
- [X] T035 [US2] Implement frontend logic to send selected text to the backend API when in selected-text mode
- [X] T036 [US2] Update frontend to clearly indicate when selected-text mode is active
- [X] T037 [US2] Ensure selected-text queries ignore unrelated book sections and focus only on the selected content

---

## Phase 5: User Story 3 - Maintain Conversation History (Priority: P3)

**Goal**: Implement persistent storage and retrieval of chat conversations to allow students to continue learning sessions across multiple visits

**Independent Test**: Can be tested by starting a chat session, asking multiple questions, leaving the site, returning, and verifying that the conversation history is preserved.

- [X] T038 [US3] Enhance database models to properly store and retrieve conversation history with chronological ordering
- [X] T039 [US3] Create database service functions to save conversation threads in Neon Postgres
- [X] T040 [US3] Implement session persistence functionality to store conversations across browser sessions
- [X] T041 [US3] Create API endpoints to retrieve conversation history for a given session
- [X] T042 [US3] Add automatic cleanup mechanism to delete chat session data after 30 days retention
- [X] T043 [US3] Implement frontend UI to display conversation history when returning to a session
- [X] T044 [US3] Add session management in frontend to maintain continuity across page visits
- [X] T045 [US3] Create functionality to clear conversation history when needed
- [X] T046 [US3] Ensure 99% reliability in chat session persistence across browser sessions

---

## Phase 6: Polish & Cross-Cutting Concerns

**Goal**: Implement additional features, error handling, and quality improvements across the application

- [X] T047 Implement comprehensive error handling for empty questions and provide appropriate user feedback
- [X] T048 Create fallback mechanisms for queries with no relevant matches in the book content
- [X] T049 Add handling for questions outside the book's domain with appropriate responses
- [X] T050 Implement protection against multiple simultaneous requests from the same session
- [X] T051 Add proper logging throughout the application for debugging and monitoring
- [X] T052 Create comprehensive tests for all API endpoints and service functions
- [X] T053 Implement proper input sanitization and security measures
- [X] T054 Optimize performance to ensure <5 second response time for queries
- [X] T055 Ensure 95% hallucination-free responses through strict context validation
- [X] T056 Add support for anonymous usage without requiring user authentication
- [X] T057 Implement graceful degradation for all external service failures
- [X] T058 Create frontend styling that ensures <10% impact on textbook page load times
- [X] T059 Add user feedback mechanisms to measure 85% satisfaction with response accuracy
- [X] T060 Conduct end-to-end testing to validate all user stories and success criteria