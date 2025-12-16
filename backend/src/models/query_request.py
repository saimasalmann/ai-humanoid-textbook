from datetime import datetime
from typing import Optional, Dict, Any
from pydantic import BaseModel, Field


class QueryRequest(BaseModel):
    """
    Model representing a query request from a user.
    """
    request_id: str = Field(..., description="Unique identifier for the request")
    session_id: str = Field(..., description="Reference to associated ChatSession")
    user_query: str = Field(..., description="Original user question")
    selected_text: Optional[str] = Field(None, description="Text selected by user (if in selected-text mode)")
    query_mode: str = Field("full-book", description="Query mode ('full-book' or 'selected-text')",
                           pattern="^(full-book|selected-text)$")
    timestamp: datetime = Field(default_factory=datetime.now, description="When the request was made")
    metadata: Optional[Dict[str, Any]] = Field(None, description="Additional metadata (chapter, section, page if available)")

    class Config:
        json_encoders = {
            datetime: lambda v: v.isoformat()
        }

    def validate_query_mode(self) -> bool:
        """
        Validate that the query mode is one of the allowed values.
        """
        return self.query_mode in ["full-book", "selected-text"]

    def validate_user_query(self) -> bool:
        """
        Validate that user_query is not empty.
        """
        return self.user_query is not None and len(self.user_query.strip()) > 0

    def validate_selected_text_requirement(self) -> bool:
        """
        Validate that selectedText is provided when queryMode is 'selected-text'.
        """
        if self.query_mode == "selected-text":
            return self.selected_text is not None and len(self.selected_text.strip()) > 0
        return True