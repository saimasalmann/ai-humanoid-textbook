from typing import List, Dict, Any, Optional
import google.generativeai as genai
from ..config.settings import settings
from ..models.retrieved_context import RetrievedContext

# Initialize the Google Generative AI client
if settings.google_api_key:
    genai.configure(api_key=settings.google_api_key)
else:
    print("WARNING: Google API key not set. Qdrant service will not function properly.")


class QdrantRAGService:
    """
    Service class to handle RAG operations with Qdrant vector database.
    This service retrieves relevant content from Qdrant based on user queries.
    """
    def __init__(self):
        from .qdrant_client import qdrant_service as client_service
        self.client_service = client_service
        self.embedding_model = "embedding-001"  # Google's embedding model
        self.is_configured = settings.google_api_key is not None and settings.google_api_key != ""

    async def retrieve_context(self, query: str, limit: int = 5) -> List[RetrievedContext]:
        """
        Retrieve relevant content from Qdrant based on user query.

        Args:
            query: The user's question or query text
            limit: Maximum number of results to return

        Returns:
            List of RetrievedContext objects with relevant content
        """
        if not self.is_configured:
            print("Qdrant service not configured - returning empty results")
            return []

        try:
            # First, generate embedding for the query using Google's embedding service
            embedding_response = genai.embed_content(
                model=self.embedding_model,
                content=[query],
                task_type="RETRIEVAL_QUERY"
            )

            query_embedding = embedding_response['embedding'][0]

            # Use the Qdrant client to search for similar content
            search_results = await self.client_service.search(
                query_vector=query_embedding,
                limit=limit
            )

            # Convert results to RetrievedContext models
            retrieved_contexts = []
            for i, result in enumerate(search_results):
                context = RetrievedContext(
                    context_id=f"context_{i}_{result.get('id', 'unknown')}",
                    request_id="temp_request_id",  # Will be set by the calling function
                    content=result.get("content", ""),
                    metadata=result.get("metadata", {}),
                    similarity_score=result.get("similarity_score", 0.0),
                    rank=i + 1
                )
                retrieved_contexts.append(context)

            return retrieved_contexts
        except Exception as e:
            print(f"Error retrieving context from Qdrant: {e}")
            return []

    async def retrieve_context_for_selected_text(self, selected_text: str, query: str, limit: int = 3) -> List[RetrievedContext]:
        """
        Retrieve relevant content specifically from the selected text context.

        Args:
            selected_text: The text selected by the user
            query: The user's question about the selected text
            limit: Maximum number of results to return

        Returns:
            List of RetrievedContext objects with relevant content
        """
        if not self.is_configured:
            print("Qdrant service not configured - returning empty results")
            return []

        # For selected-text mode, we prioritize content that's similar to the selected text
        try:
            # Create a combined query that includes both the selected text and the question
            combined_query = f"{selected_text} {query}"

            # Generate embedding for the combined query
            embedding_response = genai.embed_content(
                model=self.embedding_model,
                content=[combined_query],
                task_type="RETRIEVAL_QUERY"
            )

            query_embedding = embedding_response['embedding'][0]

            # Use the Qdrant client to search for similar content
            search_results = await self.client_service.search(
                query_vector=query_embedding,
                limit=limit
            )

            # Filter results to ensure they're relevant to the selected text
            filtered_results = []
            for i, result in enumerate(search_results):
                # In selected-text mode, we might want to apply additional filtering
                # to ensure the retrieved content is related to the selected text
                if self._is_relevant_to_selected_text(result.get("content", ""), selected_text):
                    context = RetrievedContext(
                        context_id=f"selected_context_{i}_{result.get('id', 'unknown')}",
                        request_id="temp_request_id",  # Will be set by the calling function
                        content=result.get("content", ""),
                        metadata=result.get("metadata", {}),
                        similarity_score=result.get("similarity_score", 0.0),
                        rank=i + 1
                    )
                    filtered_results.append(context)

            return filtered_results
        except Exception as e:
            print(f"Error retrieving context for selected text from Qdrant: {e}")
            return []

    def _is_relevant_to_selected_text(self, content: str, selected_text: str) -> bool:
        """
        Simple heuristic to check if content is relevant to selected text.
        In a real implementation, this would be more sophisticated.
        """
        # Convert both to lowercase for comparison
        content_lower = content.lower()
        selected_lower = selected_text.lower()

        # Check if key terms from selected text appear in content
        selected_words = selected_lower.split()
        if len(selected_words) == 0:
            return True

        # Count how many words from selected text appear in content
        matching_words = sum(1 for word in selected_words if word in content_lower)
        match_ratio = matching_words / len(selected_words)

        # Consider relevant if at least 30% of words match
        return match_ratio >= 0.3


# Global Qdrant RAG service instance
qdrant_rag_service = QdrantRAGService()