try:
    from qdrant_client import QdrantClient
    from qdrant_client.http.models import SearchRequest, Distance, VectorParams
    QDRANT_AVAILABLE = True
except ImportError:
    QDRANT_AVAILABLE = False
    QdrantClient = None

from typing import List, Dict, Any, Optional
from ..config.settings import settings


class QdrantService:
    """
    Service class to handle connections and operations with Qdrant vector database.
    """
    def __init__(self):
        if QDRANT_AVAILABLE and settings.qdrant_url and settings.qdrant_api_key:
            self.client = QdrantClient(
                url=settings.qdrant_url,
                api_key=settings.qdrant_api_key,
                https=True
            )
            self.is_configured = True
        else:
            self.client = None
            self.is_configured = False
            if not QDRANT_AVAILABLE:
                print("WARNING: qdrant-client not available. Qdrant service will not function properly.")
            elif not settings.qdrant_url:
                print("WARNING: QDRANT_URL not set. Qdrant service will not function properly.")
            elif not settings.qdrant_api_key:
                print("WARNING: QDRANT_API_KEY not set. Qdrant service will not function properly.")

        self.collection_name = "ai-book"  # Default collection name

    async def search(self, query_vector: List[float], limit: int = 10) -> List[Dict[str, Any]]:
        """
        Perform a vector search in Qdrant.

        Args:
            query_vector: The vector to search for
            limit: Maximum number of results to return

        Returns:
            List of search results with content, metadata, and similarity scores
        """
        if not self.is_configured:
            print("Qdrant service not configured - returning empty results")
            return []

        try:
            search_result = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=limit
            )

            results = []
            for point in search_result:
                result = {
                    "content": point.payload.get("content", ""),
                    "metadata": point.payload.get("metadata", {}),
                    "similarity_score": point.score,
                    "id": point.id
                }
                results.append(result)

            return results
        except Exception as e:
            print(f"Error searching Qdrant: {e}")
            return []

    async def search_with_text_query(self, text_query: str, limit: int = 5) -> List[Dict[str, Any]]:
        """
        Perform a search using text query (requires embedding).
        This is a simplified version - in a real implementation, you'd convert the text
        to a vector using an embedding model first.

        Args:
            text_query: The text to search for
            limit: Maximum number of results to return

        Returns:
            List of search results with content, metadata, and similarity scores
        """
        if not self.is_configured:
            print("Qdrant service not configured - returning empty results")
            return []

        # In a real implementation, you would:
        # 1. Use an embedding model to convert text_query to a vector
        # 2. Call the search method with that vector
        # For now, this is a placeholder implementation
        print(f"Text search not fully implemented yet for: {text_query}")
        return []

    def get_collection_info(self) -> Dict[str, Any]:
        """
        Get information about the collection.
        """
        if not self.is_configured:
            print("Qdrant service not configured - returning empty info")
            return {}

        try:
            collection_info = self.client.get_collection(self.collection_name)
            return {
                "name": collection_info.config.params.vectors_count,
                "vectors_count": collection_info.vectors_count,
                "indexed_vectors_count": collection_info.indexed_vectors_count
            }
        except Exception as e:
            print(f"Error getting collection info: {e}")
            return {}


# Global Qdrant service instance
qdrant_service = QdrantService()