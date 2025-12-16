import re
from typing import Optional
from datetime import datetime, timedelta
from ..models.message import Message
from ..models.query_request import QueryRequest


def validate_session_id(session_id: str) -> bool:
    """
    Validate session ID format.
    A valid session ID should be a non-empty string with reasonable length.
    """
    if not session_id or not isinstance(session_id, str):
        return False
    return 1 <= len(session_id) <= 255


def validate_user_id(user_id: str) -> bool:
    """
    Validate user ID format for anonymous users.
    """
    if not user_id or not isinstance(user_id, str):
        return True  # user_id is optional, so None/empty is valid
    return 1 <= len(user_id) <= 255


def validate_message_content(content: str) -> bool:
    """
    Validate message content.
    """
    if not content or not isinstance(content, str):
        return False
    return 1 <= len(content) <= 10000  # Max 10k characters


def validate_selected_text(selected_text: str) -> bool:
    """
    Validate selected text.
    """
    if selected_text is None:
        return True  # selected_text is optional
    if not isinstance(selected_text, str):
        return False
    return 1 <= len(selected_text) <= 10000  # Max 10k characters


def validate_query_mode(query_mode: str) -> bool:
    """
    Validate query mode.
    """
    if not query_mode or not isinstance(query_mode, str):
        return False
    return query_mode in ["full-book", "selected-text"]


def validate_role(role: str) -> bool:
    """
    Validate message role.
    """
    if not role or not isinstance(role, str):
        return False
    return role in ["user", "assistant", "system"]


def validate_message_model(message: Message) -> bool:
    """
    Validate a Message model instance.
    """
    return (
        validate_session_id(message.session_id) and
        validate_role(message.role) and
        validate_message_content(message.content) and
        validate_query_mode(message.query_mode) and
        validate_selected_text(message.selected_text) and
        (message.query_mode != "selected-text" or validate_selected_text(message.selected_text))
    )


def validate_query_request_model(query_request: QueryRequest) -> bool:
    """
    Validate a QueryRequest model instance.
    """
    return (
        validate_session_id(query_request.session_id) and
        validate_message_content(query_request.user_query) and
        validate_query_mode(query_request.query_mode) and
        validate_selected_text(query_request.selected_text) and
        (query_request.query_mode != "selected-text" or validate_selected_text(query_request.selected_text))
    )


def validate_session_expiration(expires_at: datetime) -> bool:
    """
    Validate that session expiration is in the future.
    """
    if not isinstance(expires_at, datetime):
        return False
    return expires_at > datetime.now()


def is_valid_uuid(uuid_string: str) -> bool:
    """
    Check if a string is a valid UUID.
    """
    uuid_pattern = re.compile(
        r'^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$',
        re.I
    )
    return bool(uuid_pattern.match(uuid_string))


def sanitize_input(text: str) -> str:
    """
    Basic input sanitization to prevent common injection attacks.
    """
    if not text:
        return text

    # Remove null bytes
    text = text.replace('\x00', '')

    # Remove control characters except common whitespace
    text = ''.join(char for char in text if ord(char) >= 32 or char in '\t\n\r')

    return text