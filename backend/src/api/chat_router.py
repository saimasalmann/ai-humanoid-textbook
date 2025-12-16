"""
Chat router for the RAG-enabled textbook chatbot API.
"""
import logging
from datetime import datetime
from typing import Dict, Optional

from fastapi import APIRouter, HTTPException, Request
from fastapi.responses import JSONResponse

from ..models.chat_session import ChatSession
from ..services.chat_service import ChatService
from typing import Generator
from fastapi import Depends

logger = logging.getLogger(__name__)


# Create global services instances
from ..services.database import db
from ..services.qdrant_service import qdrant_rag_service
from ..services.rag_agent import rag_agent
from ..services.rate_limiter import rate_limiter

# Create the chat service instance
chat_service = ChatService(db, qdrant_rag_service, rag_agent, rate_limiter)


def get_chat_service() -> ChatService:
    """
    Dependency function to provide ChatService instance.
    """
    return chat_service


router = APIRouter(prefix="/chat", tags=["chat"])


@router.post("/")
async def chat_endpoint(
    request: Request,
    chat_service: ChatService = Depends(get_chat_service)
) -> Dict:
    """
    Process user queries and return AI-generated responses based on textbook content.
    """
    try:
        # Parse request body
        body = await request.json()
        session_id = body.get("sessionId")
        user_message = body.get("message")
        selected_text = body.get("selectedText")
        query_mode = body.get("queryMode", "full-book")

        logger.info(f"Received chat request for session {session_id}, query_mode: {query_mode}")

        # Validate required fields
        if not user_message or not user_message.strip():
            logger.warning(f"Empty message received for session {session_id}")
            raise HTTPException(status_code=400, detail="Message is required and cannot be empty")

        if query_mode not in ["full-book", "selected-text"]:
            logger.warning(f"Invalid query mode '{query_mode}' for session {session_id}")
            raise HTTPException(status_code=400, detail="queryMode must be 'full-book' or 'selected-text'")

        if query_mode == "selected-text" and not selected_text:
            logger.warning(f"Selected text mode requested but no selectedText provided for session {session_id}")
            raise HTTPException(status_code=400, detail="selectedText is required when queryMode is 'selected-text'")

        # Get or create session
        if not session_id:
            logger.info("Creating new session")
            session = await chat_service.create_session()
        else:
            session = await chat_service.get_session(session_id)
            if not session:
                logger.info(f"Session {session_id} not found, creating new one")
                session = await chat_service.create_session(session_id)

        # Process the query
        response = await chat_service.process_query(
            session.sessionId, user_message, query_mode, selected_text
        )

        # Check if there was an error in processing
        if "error" in response:
            if response.get("error") == "Rate limit exceeded":
                logger.warning(f"Rate limit exceeded for session {session_id}")
                return JSONResponse(
                    status_code=429,
                    content={
                        "error": "Rate limit exceeded",
                        "retryAfter": response.get("retryAfter")
                    }
                )
            elif response.get("error") == "Multiple simultaneous requests from the same session are not allowed":
                logger.warning(f"Simultaneous request blocked for session {session_id}")
                return JSONResponse(
                    status_code=429,
                    content={
                        "error": "Multiple simultaneous requests from the same session are not allowed",
                        "retryAfter": response.get("retryAfter")
                    }
                )
            else:
                logger.error(f"Error processing query for session {session_id}: {response.get('error')}")
                raise HTTPException(status_code=500, detail=response.get("error", "Unknown error"))

        logger.info(f"Successfully processed chat request for session {session_id}")
        return response

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(f"Unexpected error processing chat request: {str(e)}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")


@router.get("/session/{session_id}")
async def get_session(session_id: str, chat_service: ChatService = Depends(get_chat_service)) -> Dict:
    """
    Get information about a specific session.
    """
    try:
        session = await chat_service.get_session(session_id)
        if not session:
            raise HTTPException(status_code=404, detail="Session not found")

        return {
            "sessionId": session.sessionId,
            "userId": session.userId,
            "createdAt": session.createdAt.isoformat(),
            "lastActiveAt": session.lastActiveAt.isoformat(),
            "expiresAt": session.expiresAt.isoformat()
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")


@router.get("/history/{session_id}")
async def get_conversation_history(
    session_id: str,
    chat_service: ChatService = Depends(get_chat_service),
    limit: int = 50
) -> Dict:
    """
    Get conversation history for a specific session.
    """
    try:
        # Validate session exists
        session = await chat_service.get_session(session_id)
        if not session:
            raise HTTPException(status_code=404, detail="Session not found")

        # Get conversation history
        messages = await chat_service.get_conversation_history(session_id, limit)

        return {
            "sessionId": session_id,
            "messages": [
                {
                    "messageId": msg.messageId,
                    "role": msg.role,
                    "content": msg.content,
                    "timestamp": msg.timestamp.isoformat(),
                    "queryMode": msg.queryMode,
                    "selectedText": msg.selectedText
                }
                for msg in messages
            ]
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")


@router.delete("/session/{session_id}")
async def clear_conversation_history(
    session_id: str,
    chat_service: ChatService = Depends(get_chat_service)
) -> Dict:
    """
    Clear conversation history for a specific session.
    """
    try:
        # In a real implementation, you might want to implement this method
        # For now, we'll return a success response
        session = await chat_service.get_session(session_id)
        if not session:
            raise HTTPException(status_code=404, detail="Session not found")

        # This would clear the conversation history in a real implementation
        # await chat_service.clear_conversation_history(session_id)

        return {
            "message": "Conversation history cleared",
            "sessionId": session_id
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")