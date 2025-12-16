"""
Constants for the Qdrant embedding pipeline.

This module defines default values and configuration constants.
"""
# Default values for chunking
DEFAULT_CHUNK_SIZE = 348  # tokens
DEFAULT_BATCH_SIZE = 10
DEFAULT_MODEL_NAME = "models/embedding-001"

# Qdrant configuration
VECTOR_SIZE = 768  # For Google Gemini embedding-001 model
DISTANCE_FUNCTION = "Cosine"

# File processing
SUPPORTED_FILE_EXTENSIONS = {".md", ".mdx"}

# Performance and limits
MAX_FILE_SIZE_MB = 50  # Maximum file size to process (in MB)
MAX_MEMORY_USAGE_PERCENT = 80  # Maximum memory usage percentage

# Retry configuration
MAX_RETRIES = 3
RETRY_DELAY_BASE = 1  # Base delay in seconds for exponential backoff