"""
Qdrant service for the Qdrant embedding pipeline.

This module handles connecting to Qdrant and storing embeddings with metadata.
"""
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams
from qdrant_client.http.exceptions import UnexpectedResponse
from .models import DocumentChunk
from .config import Config
from .constants import VECTOR_SIZE, DISTANCE_FUNCTION
from .retry_handler import retry_with_exponential_backoff


class QdrantService:
    """Service to handle Qdrant operations."""

    def __init__(self):
        """Initialize the Qdrant service."""
        # Validate configuration before initializing client
        Config.validate_and_exit_if_invalid()

        self.client = QdrantClient(
            url=Config.QDRANT_URL,
            api_key=Config.QDRANT_API_KEY,
            prefer_grpc=False  # Use REST API
        )
        self.collection_name = Config.COLLECTION_NAME

    @retry_with_exponential_backoff(
        max_retries=3,
        base_delay=1,
        max_delay=60,
        exceptions=(Exception,)
    )
    def ensure_collection_exists(self) -> bool:
        """
        Check if the collection exists, create it if it doesn't.

        Returns:
            bool: True if collection exists or was created successfully
        """
        # Try to get collection info to check if it exists
        self.client.get_collection(self.collection_name)
        return True

    @retry_with_exponential_backoff(
        max_retries=3,
        base_delay=1,
        max_delay=60,
        exceptions=(Exception,)
    )
    def create_collection(self) -> bool:
        """
        Create a new Qdrant collection with the appropriate vector size and distance function.

        Returns:
            bool: True if collection was created successfully
        """
        self.client.create_collection(
            collection_name=self.collection_name,
            vectors_config=VectorParams(
                size=VECTOR_SIZE,
                distance=Distance.COSINE
            )
        )
        return True

    def store_embeddings_batch(
        self,
        chunks: List[DocumentChunk],
        batch_size: int = 10
    ) -> bool:
        """
        Store embeddings with metadata in Qdrant in batches.

        Args:
            chunks (List[DocumentChunk]): List of chunks with embeddings to store
            batch_size (int): Number of vectors to store in each batch

        Returns:
            bool: True if all embeddings were stored successfully
        """
        if not chunks:
            return True

        # Process in batches
        for i in range(0, len(chunks), batch_size):
            batch = chunks[i:i + batch_size]
            success = self._store_batch(batch)
            if not success:
                return False

        return True

    @retry_with_exponential_backoff(
        max_retries=3,
        base_delay=1,
        max_delay=60,
        exceptions=(Exception,)
    )
    def _store_batch(self, chunks: List[DocumentChunk]) -> bool:
        """
        Store a single batch of embeddings in Qdrant.

        Args:
            chunks (List[DocumentChunk]): Batch of chunks to store

        Returns:
            bool: True if batch was stored successfully
        """
        # Prepare points for insertion
        points = []
        for chunk in chunks:
            if chunk.vector is None:
                print(f"Warning: Chunk {chunk.id} has no vector, skipping")
                continue

            # Create payload with metadata
            payload = {
                "content": chunk.content,
                "file_path": chunk.file_path,
                "chunk_index": chunk.chunk_index,
                "chapter": chunk.chapter,
                "created_at": chunk.created_at.isoformat() if chunk.created_at else None
            }

            # Create point
            point = models.PointStruct(
                id=chunk.id,
                vector=chunk.vector,
                payload=payload
            )
            points.append(point)

        if not points:
            return True  # Nothing to store

        # Insert points into collection
        self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )

        return True

    def validate_storage(self, expected_count: int) -> bool:
        """
        Validate that the expected number of embeddings were stored.

        Args:
            expected_count (int): Expected number of stored embeddings

        Returns:
            bool: True if count matches expected value
        """
        try:
            collection_info = self.client.get_collection(self.collection_name)
            actual_count = collection_info.points_count
            return actual_count == expected_count
        except Exception as e:
            print(f"Error validating storage: {e}")
            return False

    def search_similar(self, query_vector: List[float], limit: int = 5) -> List[Dict[str, Any]]:
        """
        Search for similar vectors in the collection.

        Args:
            query_vector (List[float]): The query vector for similarity search
            limit (int): Maximum number of results to return

        Returns:
            List[Dict[str, Any]]: List of similar points with payload
        """
        try:
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=limit
            )
            return [
                {
                    "id": result.id,
                    "payload": result.payload,
                    "score": result.score
                }
                for result in results
            ]
        except Exception as e:
            print(f"Error searching for similar vectors: {e}")
            return []