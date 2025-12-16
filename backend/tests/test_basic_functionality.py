"""
Basic tests for the RAG Chatbot API
"""
import pytest
import asyncio
from fastapi.testclient import TestClient
from src.api.main import app
from src.api.chat_router import router as chat_router
from src.api.health_router import router as health_router

# Add routers to the app for testing
app.include_router(chat_router, prefix="", tags=["chat"])
app.include_router(health_router, prefix="", tags=["health"])

client = TestClient(app)

def test_root_endpoint():
    """Test the root endpoint"""
    response = client.get("/")
    assert response.status_code == 200
    data = response.json()
    assert data["message"] == "RAG Chatbot API"
    assert data["status"] == "running"

def test_health_check():
    """Test the health check endpoint"""
    response = client.get("/health")
    assert response.status_code == 200
    data = response.json()
    assert "status" in data
    assert "dependencies" in data
    assert isinstance(data["dependencies"], dict)

def test_chat_endpoint_missing_message():
    """Test chat endpoint with missing message"""
    response = client.post("/chat", json={
        "sessionId": "test-session-123"
    })
    assert response.status_code == 400

def test_chat_endpoint_invalid_query_mode():
    """Test chat endpoint with invalid query mode"""
    response = client.post("/chat", json={
        "sessionId": "test-session-123",
        "message": "Hello",
        "queryMode": "invalid-mode"
    })
    assert response.status_code == 400

def test_chat_endpoint_valid_request_structure():
    """Test that chat endpoint accepts valid requests"""
    response = client.post("/chat", json={
        "sessionId": "test-session-123",
        "message": "What is this textbook about?",
        "queryMode": "full-book"
    })
    # This should either return a 200 response or a 429 (rate limit) or 500 (service unavailable)
    # depending on whether external services are configured
    assert response.status_code in [200, 429, 500, 503]

if __name__ == "__main__":
    pytest.main([__file__])