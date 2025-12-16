import logging
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from .health_router import router as health_router
from .chat_router import router as chat_router
from ..config.settings import settings

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


# Create FastAPI app instance
app = FastAPI(
    title="RAG Chatbot API",
    description="API for RAG-enabled textbook chatbot",
    version="1.0.0"
)


# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# Include health router
app.include_router(health_router, prefix="", tags=["health"])

# Include chat router
app.include_router(chat_router, prefix="", tags=["chat"])


@app.on_event("startup")
async def startup_event():
    """
    Startup event to initialize connections.
    """
    logger.info("Starting up RAG Chatbot API...")
    # Initialize database connection
    # await db.connect()  # Uncomment when database is configured
    # Initialize Redis for rate limiting
    # await rate_limiter.init_redis()  # Uncomment when Redis is configured


@app.on_event("shutdown")
async def shutdown_event():
    """
    Shutdown event to clean up connections.
    """
    logger.info("Shutting down RAG Chatbot API...")
    # Close database connection
    # await db.disconnect()  # Uncomment when database is configured


@app.get("/")
async def root():
    """
    Root endpoint for basic API information.
    """
    return {
        "message": "RAG Chatbot API",
        "version": "1.0.0",
        "status": "running"
    }