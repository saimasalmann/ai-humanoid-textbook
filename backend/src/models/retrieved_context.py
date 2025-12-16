from typing import Dict, Any
from pydantic import BaseModel, Field


class RetrievedContext(BaseModel):
    """
    Model representing context retrieved from Qdrant based on user query.
    """
    context_id: str = Field(..., description="Unique identifier for the retrieved context")
    request_id: str = Field(..., description="Reference to associated QueryRequest")
    content: str = Field(..., description="Retrieved text content from Qdrant")
    metadata: Dict[str, Any] = Field(..., description="Metadata from Qdrant (chapter, section, page, etc.)")
    similarity_score: float = Field(..., description="Similarity score from vector search", ge=0.0, le=1.0)
    rank: int = Field(..., description="Rank in the retrieval results", ge=1)

    class Config:
        # Additional configuration if needed
        pass

    def validate_similarity_score(self) -> bool:
        """
        Validate that similarityScore is between 0 and 1.
        """
        return 0.0 <= self.similarity_score <= 1.0

    def validate_rank(self) -> bool:
        """
        Validate that rank is a positive integer.
        """
        return self.rank > 0