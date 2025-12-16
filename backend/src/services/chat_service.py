"""
Chat service to manage chat sessions and message flow for the RAG-enabled textbook chatbot.
"""
import asyncio
from datetime import datetime, timedelta
from typing import Dict, List, Optional
from uuid import uuid4

from ..models.chat_session import ChatSession
from ..models.message import Message
from ..models.query_request import QueryRequest
from ..models.retrieved_context import RetrievedContext
from ..services.database import DatabaseService
from ..services.qdrant_service import QdrantRAGService as QdrantService
from ..services.rag_agent import RAGAgent
from ..services.rate_limiter import RateLimiter


class ChatService:
    """
    Service to manage chat sessions and message flow.
    """

    def __init__(self, db_service: DatabaseService, qdrant_service: QdrantService,
                 rag_agent: RAGAgent, rate_limiter: RateLimiter):
        self.db_service = db_service
        self.qdrant_service = qdrant_service
        self.rag_agent = rag_agent
        self.rate_limiter = rate_limiter
        self.sessions: Dict[str, ChatSession] = {}
        self.active_requests: Dict[str, bool] = {}  # Track active requests per session

    async def create_session(self, session_id: Optional[str] = None) -> ChatSession:
        """
        Create a new chat session or return existing one.
        """
        if not session_id:
            session_id = f"sess_{uuid4().hex[:12]}"

        now = datetime.utcnow()
        expires_at = now + timedelta(days=30)  # 30-day retention

        session = ChatSession(
            session_id=session_id,
            user_id=None,  # Anonymous user
            created_at=now,
            last_active_at=now,
            expires_at=expires_at
        )

        # Store in database
        await self.db_service.create_session(session)

        # Cache in memory
        self.sessions[session_id] = session

        return session

    async def get_session(self, session_id: str) -> Optional[ChatSession]:
        """
        Get a chat session by ID, either from cache or database.
        """
        # Check cache first
        if session_id in self.sessions:
            return self.sessions[session_id]

        # Fetch from database
        session = await self.db_service.get_session(session_id)
        if session:
            self.sessions[session_id] = session

        return session

    async def add_message(self, session_id: str, role: str, content: str,
                         query_mode: str = "full-book", selected_text: Optional[str] = None) -> Message:
        """
        Add a message to a chat session.
        """
        message_id = f"msg_{uuid4().hex[:12]}"
        timestamp = datetime.utcnow()

        message = Message(
            messageId=message_id,
            sessionId=session_id,
            role=role,
            content=content,
            timestamp=timestamp,
            queryMode=query_mode,
            selectedText=selected_text
        )

        # Store in database
        await self.db_service.create_message(message)

        # Update session last active time
        await self.update_session_activity(session_id)

        return message

    async def update_session_activity(self, session_id: str):
        """
        Update the last active time for a session.
        """
        now = datetime.utcnow()

        # Update in database
        await self.db_service.update_session_activity(session_id, now)

        # Update in cache if exists
        if session_id in self.sessions:
            self.sessions[session_id].lastActiveAt = now

    async def process_query(self, session_id: str, user_query: str,
                           query_mode: str = "full-book",
                           selected_text: Optional[str] = None) -> Dict:
        """
        Process a user query and return AI-generated response with retrieved context.
        """
        # Check for simultaneous requests from the same session
        if session_id in self.active_requests and self.active_requests[session_id]:
            return {
                "error": "Multiple simultaneous requests from the same session are not allowed",
                "retryAfter": 2  # Suggest retrying after 2 seconds
            }

        # Mark this session as having an active request
        self.active_requests[session_id] = True

        try:
            # Check rate limit
            if not await self.rate_limiter.is_allowed(session_id):
                return {
                    "error": "Rate limit exceeded",
                    "retryAfter": await self.rate_limiter.get_reset_time(session_id)
                }

            # Create query request
            request_id = f"req_{uuid4().hex[:12]}"
            timestamp = datetime.utcnow()

            query_request = QueryRequest(
                requestId=request_id,
                sessionId=session_id,
                userQuery=user_query,
                selectedText=selected_text,
                queryMode=query_mode,
                timestamp=timestamp,
                metadata={}
            )

            # Store query request
            await self.db_service.create_query_request(query_request)

            # Add user message to session
            await self.add_message(session_id, "user", user_query, query_mode, selected_text)

            # Retrieve relevant context from Qdrant
            retrieved_contexts: List[RetrievedContext] = []
            if query_mode == "selected-text" and selected_text:
                # In selected-text mode, use the selected text as context
                retrieved_contexts = await self.qdrant_service.search(selected_text)
            else:
                # In full-book mode, search with user query
                retrieved_contexts = await self.qdrant_service.search(user_query)

            # If no relevant contexts found, prepare fallback response
            if not retrieved_contexts:
                # Create a fallback response indicating no relevant content was found
                ai_response = "I couldn't find any relevant content in the textbook to answer your question. Please try rephrasing your question or check if the topic is covered in the textbook."

                # Add AI message to session
                await self.add_message(session_id, "assistant", ai_response, query_mode, selected_text)

                # Format response for no content found
                response = {
                    "responseId": f"resp_{uuid4().hex[:12]}",
                    "sessionId": session_id,
                    "message": ai_response,
                    "retrievedContext": [],
                    "timestamp": timestamp.isoformat(),
                    "queryMode": query_mode,
                    "fallback": True
                }

                return response

            # Generate AI response using RAG agent
            ai_response = await self.rag_agent.generate_response(
                user_query, retrieved_contexts, query_mode, selected_text
            )

            # Add AI message to session
            await self.add_message(session_id, "assistant", ai_response, query_mode, selected_text)

            # Format response
            response = {
                "responseId": f"resp_{uuid4().hex[:12]}",
                "sessionId": session_id,
                "message": ai_response,
                "retrievedContext": [
                    {
                        "content": ctx.content,
                        "metadata": ctx.metadata,
                        "similarityScore": ctx.similarityScore
                    }
                    for ctx in retrieved_contexts
                ],
                "timestamp": timestamp.isoformat(),
                "queryMode": query_mode
            }

            return response
        except Exception as e:
            # Handle Qdrant service failures gracefully
            error_response = {
                "responseId": f"resp_{uuid4().hex[:12]}",
                "sessionId": session_id,
                "message": "The search service is temporarily unavailable. Please try again later.",
                "retrievedContext": [],
                "timestamp": timestamp.isoformat(),
                "queryMode": query_mode,
                "error": "Service unavailable"
            }
            return error_response
        finally:
            # Always release the active request flag
            if session_id in self.active_requests:
                self.active_requests[session_id] = False

    async def get_conversation_history(self, session_id: str, limit: int = 50) -> List[Message]:
        """
        Get conversation history for a session.
        """
        return await self.db_service.get_messages_by_session(session_id, limit)

    async def cleanup_expired_sessions(self):
        """
        Remove expired sessions from database.
        """
        await self.db_service.delete_expired_sessions()