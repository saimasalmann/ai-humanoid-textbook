"""
Embedding service for the Qdrant embedding pipeline.

This module handles generating embeddings using Google's Gemini model.
"""
from typing import List, Optional
import google.generativeai as genai
from google.api_core import exceptions as google_exceptions
from .models import DocumentChunk, EmbeddingVector
from .config import Config
from .constants import DEFAULT_MODEL_NAME, VECTOR_SIZE
from .retry_handler import retry_with_exponential_backoff


class EmbeddingService:
    """Service to generate embeddings for document chunks."""

    def __init__(self, model_name: str = DEFAULT_MODEL_NAME):
        """
        Initialize the embedding service.

        Args:
            model_name (str): Name of the embedding model to use
        """
        self.model_name = model_name
        genai.configure(api_key=Config.GOOGLE_API_KEY)

    @retry_with_exponential_backoff(
        max_retries=3,
        base_delay=1,
        max_delay=60,
        exceptions=(google_exceptions.GoogleAPIError, google_exceptions.RetryError, TimeoutError)
    )
    def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for a single text using Google's Gemini model.

        Args:
            text (str): Text to generate embedding for

        Returns:
            List[float]: Embedding vector
        """
        response = genai.embed_content(
            model=self.model_name,
            content=text,
            task_type="RETRIEVAL_DOCUMENT"
        )
        return response['embedding']

    @retry_with_exponential_backoff(
        max_retries=3,
        base_delay=1,
        max_delay=60,
        exceptions=(google_exceptions.GoogleAPIError, google_exceptions.RetryError, TimeoutError)
    )
    def generate_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a batch of texts using Google's Gemini model.

        Args:
            texts (List[str]): List of texts to generate embeddings for

        Returns:
            List[List[float]]: List of embedding vectors
        """
        if not texts:
            return []

        embeddings = []
        for text in texts:
            try:
                # Add a small delay between requests to avoid rate limiting
                import time
                time.sleep(0.1)  # 100ms delay between requests

                response = genai.embed_content(
                    model=self.model_name,
                    content=text,
                    task_type="RETRIEVAL_DOCUMENT"
                )
                embeddings.append(response['embedding'])
            except Exception as e:
                print(f"Error generating embedding for text: {e}")
                # Add a zero vector as fallback if individual text fails
                embeddings.append([0.0] * VECTOR_SIZE)

        return embeddings

    def create_embedding_vector(
        self,
        chunk: DocumentChunk,
        embedding: List[float]
    ) -> EmbeddingVector:
        """
        Create an EmbeddingVector object from a chunk and its embedding.

        Args:
            chunk (DocumentChunk): The source document chunk
            embedding (List[float]): The embedding vector

        Returns:
            EmbeddingVector: EmbeddingVector object with metadata
        """
        return EmbeddingVector(
            vector_data=embedding,
            model_used=self.model_name,
            dimension=len(embedding),
            source_chunk_id=chunk.id
        )

    def process_chunk(self, chunk: DocumentChunk) -> tuple[DocumentChunk, EmbeddingVector]:
        """
        Process a single chunk by generating its embedding.

        Args:
            chunk (DocumentChunk): The chunk to process

        Returns:
            tuple[DocumentChunk, EmbeddingVector]: The chunk with embedding and the embedding vector
        """
        embedding = self.generate_embedding(chunk.content)
        chunk.vector = embedding
        embedding_vector = self.create_embedding_vector(chunk, embedding)
        return chunk, embedding_vector

    def process_chunks_batch(self, chunks: List[DocumentChunk]) -> List[tuple[DocumentChunk, EmbeddingVector]]:
        """
        Process a batch of chunks by generating their embeddings.

        Args:
            chunks (List[DocumentChunk]): List of chunks to process

        Returns:
            List[tuple[DocumentChunk, EmbeddingVector]]: List of processed chunks with their embeddings
        """
        if not chunks:
            return []

        # Extract texts for batch processing
        texts = [chunk.content for chunk in chunks]
        embeddings = self.generate_embeddings_batch(texts)

        # Create the result pairs
        results = []
        for i, chunk in enumerate(chunks):
            chunk.vector = embeddings[i]
            embedding_vector = self.create_embedding_vector(chunk, embeddings[i])
            results.append((chunk, embedding_vector))

        return results