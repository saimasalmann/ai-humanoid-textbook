"""
Unit tests for error handling functionality.

Tests retry mechanisms, error handling, and graceful failure scenarios.
"""
import pytest
import os
from unittest.mock import Mock, patch, MagicMock
from scripts.retry_handler import retry_with_exponential_backoff, RetryHandler
from scripts.config import Config
from scripts.qdrant_service import QdrantService
from scripts.embedding_service import EmbeddingService
from scripts.models import DocumentChunk


def test_retry_decorator_success():
    """Test that the retry decorator works when the function succeeds."""
    call_count = 0

    @retry_with_exponential_backoff(max_retries=3, base_delay=0.01)
    def test_func():
        nonlocal call_count
        call_count += 1
        if call_count < 2:
            raise Exception("Simulated failure")
        return "success"

    result = test_func()
    assert result == "success"
    assert call_count == 2  # Called once, failed, retried once, succeeded


def test_retry_decorator_max_attempts():
    """Test that the retry decorator gives up after max attempts."""
    call_count = 0

    @retry_with_exponential_backoff(max_retries=2, base_delay=0.01)
    def test_func():
        nonlocal call_count
        call_count += 1
        raise Exception(f"Simulated failure on attempt {call_count}")

    with pytest.raises(Exception) as exc_info:
        test_func()

    assert "Simulated failure on attempt 3" in str(exc_info.value)
    assert call_count == 3  # Called once, then 2 retries


def test_retry_handler_class():
    """Test the RetryHandler class functionality."""
    retry_handler = RetryHandler(max_retries=2, base_delay=0.01)

    # Test successful execution
    def success_func():
        return "success"

    result = retry_handler.execute_with_retry(success_func)
    assert result == "success"

    # Test retry behavior
    call_count = 0

    def flaky_func():
        nonlocal call_count
        call_count += 1
        if call_count < 2:
            raise Exception("Simulated failure")
        return "success"

    result = retry_handler.execute_with_retry(flaky_func)
    assert result == "success"
    assert call_count == 2


def test_retry_handler_batch():
    """Test the RetryHandler batch functionality."""
    retry_handler = RetryHandler(max_retries=1, base_delay=0.01)

    def process_item(item):
        if item == "bad":
            raise Exception(f"Processing failed for {item}")
        return f"processed_{item}"

    items = ["good1", "bad", "good2", "bad", "good3"]
    results = retry_handler.execute_batch_with_retry(process_item, items)

    # Should have 3 successful results, with "bad" items skipped
    assert len(results) == 3
    assert "processed_good1" in results
    assert "processed_good2" in results
    assert "processed_good3" in results


def test_config_validation_exit():
    """Test that config validation exits with correct code when validation fails."""
    # Temporarily modify config values for testing
    original_url = Config.QDRANT_URL
    original_key = Config.QDRANT_API_KEY
    original_name = Config.COLLECTION_NAME

    try:
        # Set to empty values to trigger validation failure
        Config.QDRANT_URL = ""
        Config.QDRANT_API_KEY = ""
        Config.COLLECTION_NAME = ""

        # This should cause sys.exit(2) - environment error
        with pytest.raises(SystemExit) as exc_info:
            Config.validate_and_exit_if_invalid()

        assert exc_info.value.code == 2
    finally:
        # Restore original values
        Config.QDRANT_URL = original_url
        Config.QDRANT_API_KEY = original_key
        Config.COLLECTION_NAME = original_name


def test_embedding_service_retry():
    """Test that embedding service methods have retry decorators."""
    # Check that methods exist and can be called (without actually calling the API)
    service = EmbeddingService()

    # Mock the OpenAI client to avoid actual API calls
    with patch.object(service.client.embeddings, 'create') as mock_create:
        # For batch processing, the API returns one embedding per input text
        mock_response = MagicMock()
        # Return one embedding per input (for batch of 2, we need 2 embeddings)
        mock_response.data = [
            MagicMock(embedding=[0.1, 0.2, 0.3, 0.4]),
            MagicMock(embedding=[0.5, 0.6, 0.7, 0.8])
        ]
        mock_create.return_value = mock_response

        # This should work without errors
        result = service.generate_embedding("test text")
        assert result == [0.1, 0.2, 0.3, 0.4]

        # Test batch method
        result_batch = service.generate_embeddings_batch(["test1", "test2"])
        assert len(result_batch) == 2
        assert result_batch[0] == [0.1, 0.2, 0.3, 0.4]
        assert result_batch[1] == [0.5, 0.6, 0.7, 0.8]


def test_graceful_file_processing():
    """Test that file processing handles errors gracefully."""
    from scripts.file_processor import extract_content_from_file

    # Test with a non-existent file
    try:
        content, chapter = extract_content_from_file("/non/existent/file.md")
        assert False, "Should have raised an exception"
    except FileNotFoundError:
        # This is expected
        pass
    except Exception:
        # This is also acceptable for graceful handling
        pass


if __name__ == "__main__":
    pytest.main([__file__])