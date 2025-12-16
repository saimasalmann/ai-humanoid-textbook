from fastapi import APIRouter
from typing import Dict, Any
from pydantic import BaseModel
from datetime import datetime
from ..services.qdrant_client import qdrant_service
from ..services.database import db
from ..services.rag_agent import rag_agent


class HealthResponse(BaseModel):
    """
    Model for health check response.
    """
    status: str
    timestamp: str
    dependencies: Dict[str, str]


router = APIRouter()


@router.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Health check endpoint to verify the status of the API and its dependencies.
    """
    timestamp = datetime.now().isoformat()

    # Check dependencies
    dependencies = {}

    # Check Qdrant connection
    try:
        info = qdrant_service.get_collection_info()
        dependencies["qdrant"] = "healthy" if info else "unhealthy"
    except Exception:
        dependencies["qdrant"] = "unhealthy"

    # Check database connection
    try:
        if db._has_asyncpg_connection():
            # Test the database connection
            if db.pool:
                async with db.pool.acquire() as conn:
                    await conn.fetchval("SELECT 1")
            dependencies["postgres"] = "healthy"
        else:
            # If no database URL provided, mark as not configured
            dependencies["postgres"] = "not configured"
    except Exception:
        dependencies["postgres"] = "unhealthy"

    # Check Google API (RAG agent) connection
    try:
        is_healthy = await rag_agent.health_check()
        dependencies["google"] = "healthy" if is_healthy else "unhealthy"
    except Exception:
        dependencies["google"] = "unhealthy"

    # Overall status
    overall_status = "healthy"
    for service_status in dependencies.values():
        if service_status == "unhealthy":
            overall_status = "unhealthy"
            break

    return HealthResponse(
        status=overall_status,
        timestamp=timestamp,
        dependencies=dependencies
    )