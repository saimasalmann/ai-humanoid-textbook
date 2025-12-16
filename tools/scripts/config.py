"""
Configuration module for the Qdrant embedding pipeline.

This module handles loading and validation of environment variables.
"""
import os
import sys
from dotenv import load_dotenv


# Load environment variables from .env file
load_dotenv()


class Config:
    """Configuration class to manage environment variables."""

    QDRANT_URL: str = os.getenv("QDRANT_URL", "")
    QDRANT_API_KEY: str = os.getenv("QDRANT_API_KEY", "")
    COLLECTION_NAME: str = os.getenv("COLLECTION_NAME", "docusaurus-embeddings")
    GOOGLE_API_KEY: str = os.getenv("GOOGLE_API_KEY", "")

    @classmethod
    def validate(cls) -> bool:
        """
        Validate that required environment variables are set.

        Returns:
            bool: True if all required variables are set, False otherwise
        """
        required_vars = [
            cls.QDRANT_URL,
            cls.QDRANT_API_KEY,
            cls.COLLECTION_NAME,
            cls.GOOGLE_API_KEY
        ]

        return all(var.strip() != "" for var in required_vars)

    @classmethod
    def get_required_vars(cls) -> list:
        """Get list of required environment variables that are missing."""
        missing = []
        if not cls.QDRANT_URL.strip():
            missing.append("QDRANT_URL")
        if not cls.QDRANT_API_KEY.strip():
            missing.append("QDRANT_API_KEY")
        if not cls.COLLECTION_NAME.strip():
            missing.append("COLLECTION_NAME")
        if not cls.GOOGLE_API_KEY.strip():
            missing.append("GOOGLE_API_KEY")
        return missing

    @classmethod
    def validate_and_exit_if_invalid(cls) -> None:
        """
        Validate environment variables and exit with appropriate error code if invalid.

        This follows the exit code specification:
        - 2: Environment error - required environment variables missing
        """
        if not cls.validate():
            missing_vars = cls.get_required_vars()
            print(f"ERROR: Missing required environment variables: {', '.join(missing_vars)}")
            print("Please set all required environment variables in your .env file or environment.")
            sys.exit(2)  # Environment error exit code