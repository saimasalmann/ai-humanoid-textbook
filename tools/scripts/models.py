"""
Data models for the Qdrant embedding pipeline.

This module defines the core data structures used throughout the application.
"""
from dataclasses import dataclass
from datetime import datetime
from typing import List, Optional, Dict, Any


@dataclass
class DocumentChunk:
    """
    A segment of content from an MD/MDX file, approximately 348 tokens in length,
    containing the text content and associated metadata.
    """
    id: str
    content: str
    file_path: str
    chunk_index: int
    chapter: str
    vector: Optional[List[float]] = None
    created_at: datetime = None

    def __post_init__(self):
        if self.created_at is None:
            self.created_at = datetime.now()


@dataclass
class EmbeddingVector:
    """
    Numerical representation of document chunk content suitable for semantic similarity search.
    """
    vector_data: List[float]
    model_used: str
    dimension: int
    source_chunk_id: str


@dataclass
class ProcessingState:
    """
    Tracks the state of the embedding process for monitoring and error recovery.
    """
    process_id: str
    total_files: int = 0
    processed_files: int = 0
    total_chunks: int = 0
    embedded_chunks: int = 0
    status: str = "running"  # running, completed, failed, paused
    start_time: datetime = None
    end_time: datetime = None
    errors: List[Dict[str, Any]] = None

    def __post_init__(self):
        if self.start_time is None:
            self.start_time = datetime.now()
        if self.errors is None:
            self.errors = []