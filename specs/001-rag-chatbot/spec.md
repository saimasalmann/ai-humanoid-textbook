# Feature Specification: RAG-Enabled AI Textbook Chatbot

**Feature Branch**: `001-rag-chatbot`
**Created**: 2025-12-14
**Status**: Draft
**Input**: User description: "RAG-Enabled AI Textbook Chatbot

Context:
The project is made with Docusaurus and deployed on vercel and all book content has been embedded into a Qdrant Cloud (free tier) vector database.

The next phase is to design and implement a Retrieval-Augmented Generation (RAG) chatbot that can be embedded inside the published textbook and answer user questions grounded strictly
in the book's content.

The system must use:
- OpenAI Agents SDK patterns with Google's Gemini model for AI orchestration (using AsyncOpenAI and OpenAIChatCompletionsModel with Google API key, not OpenAI API key)
- FastAPI as the backend API
- ChatKit Python for backend chat handling
- ChatKit.js for frontend chat UI
- Qdrant Cloud for vector retrieval
- Neon Serverless Postgres for chat/session persistence

The frontend (Docusaurus) already exists but transfer all frontend folders and files in frontend folder .
A new backend folder will be added in project root  for API and AI logic ,use uv as package manager.

---

High-Level Requirements (3 Main Parts):

1. RAG Backend & AI Agent (Core Intelligence)
- Implement an AI agent using the OpenAI Agents SDK patterns with Google's Gemini model via Google API key.
- Use AsyncOpenAI and OpenAIChatCompletionsModel to set up the client and model without requiring OpenAI API key.
- The agent must retrieve relevant content from Qdrant based on user queries using a dedicated retrieval tool.
- The agent must strictly answer questions using only retrieved book content.
- The agent must support two modes:
  - Full-book RAG mode (search entire book)
  - Selected-text mode, where answers are generated only from user-selected text on the page.
- Strong guardrails must be enforced to prevent hallucinations.
- Retrieval logic should support metadata such as chapter or section when available.

Success Criteria:
- The agent never answers outside provided context.
- The agent correctly handles both full-book and selected-text queries.
- Retrieved content is clearly separated from model reasoning.

---

2. API & Chat Infrastructure (Execution Layer)
- Build a FastAPI backend located in a new `backend/` folder.
- Expose a `/chat` endpoint that accepts:
  - user message
  - optional selected text
  - session identifier
- Integrate ChatKit Python for managing chat sessions and message flow.
- Store chat history and session data in Neon Serverless Postgres.
- The API should act as the single interface between frontend and AI agent.
- Backend must be deployable independently (e.g., Render or Railway).

Success Criteria:
- API successfully returns AI responses for valid queries.
- Chat sessions persist across requests.


---

3. Frontend Chat Experience (User Interface Integration)
- Embed a chatbot UI inside the Docusaurus textbook using ChatKit.js.
- The chatbot must allow users to:
  - Ask general questions about the book.
  - Select text on a page and ask questions only about that selected text.
- The frontend must send structured requests to the FastAPI backend.
- UI should be non-intrusive (floating widget or sidebar).
- The book build and deployment process must remain unchanged.

Success Criteria:
- Chatbot is visible and usable on the published  site.
- Selected-text questions correctly limit AI responses.
- Chat experience is responsive and intuitive.

---

Constraints:
- Frontend and backend must be separated logically (Docusaurus remains frontend, new backend folder added).
- No hallucinated answers outside book content.
- Qdrant is the single source of truth for book knowledge.
- The solution must be modular, production-ready, and deployment-friendly.

Outcome:
A fully functional AI-powered textbook where users can interact with a context-aware chatbot that answers questions grounded strictly in the book's content, both globally and based on selected text, using a clean three-part architecture."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Ask General Questions About Book Content (Priority: P1)

As a student reading the textbook, I want to ask questions about the book content so that I can get immediate clarification and deepen my understanding without leaving the textbook page.

**Why this priority**: This is the core functionality that provides immediate value to students by offering instant access to information from the textbook, enhancing their learning experience.

**Independent Test**: Can be fully tested by asking various questions about book content and verifying that the AI responds with accurate information from the textbook. Delivers immediate value by allowing students to get answers without searching through pages.

**Acceptance Scenarios**:

1. **Given** a student is viewing the textbook, **When** they type a question in the chat interface, **Then** the system retrieves relevant content from the book and generates an accurate response based on that content
2. **Given** a student asks a question about specific topics in the book, **When** the question is submitted, **Then** the system returns a response that is grounded in the book's content with no hallucinated information

---

### User Story 2 - Ask Questions About Selected Text (Priority: P2)

As a student reading the textbook, I want to select specific text on a page and ask questions only about that selected text so that I can get focused explanations without broader context interference.

**Why this priority**: This enhances the learning experience by allowing contextual questioning, which is particularly valuable for complex passages that require deeper analysis.

**Independent Test**: Can be tested by selecting text on a page, asking a question about it, and verifying that the AI response is limited to the selected text context rather than drawing from the entire book.

**Acceptance Scenarios**:

1. **Given** a student has selected text on a textbook page, **When** they ask a question in the chat interface, **Then** the system generates a response based only on the selected text
2. **Given** a student selects text and asks a question, **When** the query is processed, **Then** the response contains information exclusively from the selected text with no additional book content

---

### User Story 3 - Maintain Conversation History (Priority: P3)

As a student using the textbook, I want my chat conversations to be saved and accessible so that I can continue learning sessions across multiple visits.

**Why this priority**: This provides continuity for longer study sessions and allows students to revisit previous questions and answers for review purposes.

**Independent Test**: Can be tested by starting a chat session, asking multiple questions, leaving the site, returning, and verifying that the conversation history is preserved.

**Acceptance Scenarios**:

1. **Given** a student has an ongoing chat session, **When** they close the browser and return later, **Then** their previous conversation history is available
2. **Given** multiple questions have been asked in a session, **When** the student revisits the page, **Then** they can see the complete conversation thread

---

### Edge Cases

- What happens when a user submits an empty question?
- How does the system handle queries that have no relevant matches in the book content?
- What occurs when the Qdrant vector database is temporarily unavailable? (Handled through graceful degradation)
- How does the system behave when a user tries to ask questions outside the book's domain?
- What happens when multiple simultaneous requests are made from the same session?
- How does the system respond when a user exceeds the rate limit of 10 requests per minute?
- What happens when the Neon Serverless Postgres database is temporarily unavailable? (Handled through graceful degradation)

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST provide a floating chat widget interface embedded in the textbook that allows students to ask questions about book content
- **FR-002**: System MUST retrieve relevant content from the Qdrant vector database based on user queries using a dedicated retrieval tool passed to the AI agent
- **FR-003**: System MUST ensure AI responses are grounded only in retrieved book content with no hallucinations, using Google's Gemini model via OpenAI Agents SDK patterns
- **FR-004**: System MUST support two query modes: full-book RAG (search entire book) and selected-text mode (search only selected text)
- **FR-005**: System MUST maintain chat session history and store conversations in Neon Serverless Postgres with automatic deletion after 30 days
- **FR-006**: System MUST expose an API endpoint that accepts user messages, optional selected text, and session identifiers
- **FR-007**: System MUST display retrieved content separately from AI-generated responses to maintain transparency
- **FR-008**: System MUST handle user-selected text context when provided in the query
- **FR-009**: System MUST preserve the existing Docusaurus structure without requiring changes to the textbook build process
- **FR-010**: System MUST provide real-time chat experience with responsive UI interactions
- **FR-011**: System MUST implement rate limiting of 10 requests per minute per session
- **FR-012**: System MUST support anonymous usage without requiring user authentication
- **FR-013**: System MUST handle external service failures through graceful degradation
- **FR-014**: System MUST use AsyncOpenAI and OpenAIChatCompletionsModel to interface with Google's Gemini model using Google API key without requiring OpenAI API key

### Key Entities

- **Chat Session**: Represents a conversation between a student and the AI, containing metadata such as session ID, creation time, and user identifier
- **Query Request**: Contains user input (question), optional selected text, session identifier, and query mode (full-book or selected-text)
- **Retrieved Context**: Relevant book content retrieved from Qdrant vector database based on the user's query
- **AI Response**: The generated answer from the AI agent, based on retrieved context and user query
- **Conversation History**: Chronological record of all exchanges within a chat session

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Students can ask questions and receive accurate answers from the textbook content within 5 seconds
- **SC-002**: 95% of AI responses contain information exclusively from the book content with zero hallucinations
- **SC-003**: Students can successfully use both full-book RAG mode and selected-text mode with 98% accuracy in mode selection
- **SC-004**: 90% of students successfully complete their information-seeking tasks on first attempt
- **SC-005**: Chat sessions persist across browser sessions with 99% reliability
- **SC-006**: The embedded chat interface does not negatively impact textbook page load times by more than 10%
- **SC-007**: Students report 85% satisfaction with the accuracy and relevance of AI responses

## Clarifications *(optional)*

### Session 2025-12-14

- Q: What UI approach should be used for the chat interface? → A: Floating widget
- Q: What authentication method should be used for student access? → A: No authentication required
- Q: How should the system handle external service (Qdrant/Postgres) failures? → A: Graceful degradation
- Q: How long should chat session data be retained? → A: 30 days retention
- Q: What rate limiting should be applied to user requests? → A: 10 requests per minute

### Session 2025-12-15

- Q: How should the OpenAI Agents SDK be implemented with Google's Gemini model? → A: Use AsyncOpenAI and OpenAIChatCompletionsModel to set up the client and model with Google API key without requiring OpenAI API key
- Q: How should Qdrant data retrieval be integrated with the agent? → A: Create a dedicated tool for retrieving data from Qdrant that is passed to the agent
- Q: Which Google Gemini model should be used? → A: Use gemini-1.5-flash model via the OpenAI-compatible interface
