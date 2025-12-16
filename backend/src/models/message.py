from datetime import datetime
from typing import Optional
from pydantic import BaseModel, Field


class Message(BaseModel):
    """
    Model representing a message in a chat session.
    """
    message_id: str = Field(..., description="Unique identifier for the message")
    session_id: str = Field(..., description="Reference to parent ChatSession")
    role: str = Field(..., description="Message role (user, assistant, system)",
                     pattern="^(user|assistant|system)$")
    content: str = Field(..., description="Message content", max_length=10000)
    timestamp: datetime = Field(default_factory=datetime.now, description="When the message was created")
    query_mode: str = Field("full-book", description="Query mode ('full-book' or 'selected-text')",
                           pattern="^(full-book|selected-text)$")
    selected_text: Optional[str] = Field(None, description="Text selected by user (if in selected-text mode)")

    class Config:
        json_encoders = {
            datetime: lambda v: v.isoformat()
        }

    def validate_role(self) -> bool:
        """
        Validate that the role is one of the allowed values.
        """
        return self.role in ["user", "assistant", "system"]

    def validate_query_mode(self) -> bool:
        """
        Validate that the query mode is one of the allowed values.
        """
        return self.query_mode in ["full-book", "selected-text"]

    def validate_selected_text_requirement(self) -> bool:
        """
        Validate that selectedText is provided when queryMode is 'selected-text'.
        """
        if self.query_mode == "selected-text":
            return self.selected_text is not None and len(self.selected_text.strip()) > 0
        return True