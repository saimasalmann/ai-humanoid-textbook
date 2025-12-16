from pydantic_settings import BaseSettings
from typing import Optional


class Settings(BaseSettings):
    """
    Application settings loaded from environment variables.
    """
    google_api_key: str  # Required
    qdrant_url: str      # Required
    qdrant_api_key: Optional[str] = None
    collection_name: str  # Required
    neon_database_url: str  # Required
    redis_url: str = "redis://localhost:6379"
    rate_limit_requests: int = 10
    rate_limit_window: int = 60  # in seconds
    session_retention_days: int = 30

    class Config:
        env_file = "../.env"  # Look for .env in backend root
        env_file_encoding = 'utf-8'


# Create a singleton instance of settings
settings = Settings()