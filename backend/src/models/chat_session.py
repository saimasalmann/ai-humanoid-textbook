from datetime import datetime
from typing import Optional
from pydantic import BaseModel, Field


class ChatSession(BaseModel):
    """
    Model representing a chat session between a student and the AI.
    """
    session_id: str = Field(..., description="Unique identifier for the chat session")
    user_id: Optional[str] = Field(None, description="Anonymous user identifier (for tracking purposes)")
    created_at: datetime = Field(default_factory=datetime.now, description="Timestamp when session was created")
    last_active_at: datetime = Field(default_factory=datetime.now, description="Timestamp of last activity")
    expires_at: datetime = Field(..., description="Expiration timestamp (30 days from last activity)")

    class Config:
        json_encoders = {
            datetime: lambda v: v.isoformat()
        }

    def is_expired(self) -> bool:
        """
        Check if the session has expired.
        """
        return datetime.now() >= self.expires_at

    def extend_session(self, days: int = 30) -> None:
        """
        Extend the session expiration time.
        """
        self.expires_at = datetime.now().replace(day=datetime.now().day + days)