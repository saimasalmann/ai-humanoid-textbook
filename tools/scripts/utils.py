"""
Utility functions for the Qdrant embedding pipeline.

This module contains helper functions for token counting, logging, and other utilities.
"""
import logging
import tiktoken
from typing import List


def count_tokens(text: str, model_name: str = "gpt-3.5-turbo") -> int:
    """
    Count the number of tokens in a text string using tiktoken.

    Args:
        text (str): The input text to count tokens for
        model_name (str): The model name to use for tokenization

    Returns:
        int: The number of tokens in the text
    """
    try:
        encoding = tiktoken.encoding_for_model(model_name)
        tokens = encoding.encode(text)
        return len(tokens)
    except Exception:
        # Fallback to a rough estimation if tiktoken fails
        # This is roughly 4 characters per token
        return len(text) // 4


def setup_logging(level: str = "INFO") -> logging.Logger:
    """
    Set up logging configuration with proper formatting.

    Args:
        level (str): The logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)

    Returns:
        logging.Logger: Configured logger instance
    """
    logger = logging.getLogger("qdrant-embedding")
    logger.setLevel(getattr(logging, level.upper()))

    # Avoid adding multiple handlers if logger already has handlers
    if logger.handlers:
        return logger

    # Create console handler with formatting
    console_handler = logging.StreamHandler()
    console_handler.setLevel(getattr(logging, level.upper()))

    # Create formatter
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    console_handler.setFormatter(formatter)

    # Add handler to logger
    logger.addHandler(console_handler)

    return logger


def chunk_text(text: str, max_tokens: int = 348, model_name: str = "gpt-3.5-turbo") -> List[str]:
    """
    Split text into chunks that don't exceed the maximum token count.

    Args:
        text (str): The input text to chunk
        max_tokens (int): Maximum number of tokens per chunk
        model_name (str): The model name to use for tokenization

    Returns:
        List[str]: List of text chunks
    """
    # First, try to split by paragraphs to maintain semantic coherence
    paragraphs = text.split('\n\n')
    chunks = []
    current_chunk = ""

    for paragraph in paragraphs:
        # Check if adding this paragraph would exceed the token limit
        test_chunk = current_chunk + "\n\n" + paragraph if current_chunk else paragraph

        if count_tokens(test_chunk, model_name) <= max_tokens:
            current_chunk = test_chunk
        else:
            # If current chunk is not empty, save it before processing the new paragraph
            if current_chunk:
                chunks.append(current_chunk.strip())

            # If the single paragraph is too long, we need to split it further
            if count_tokens(paragraph, model_name) > max_tokens:
                # Split the long paragraph into sentences and process them
                sentences = paragraph.split('. ')
                temp_chunk = ""

                for sentence in sentences:
                    sentence = sentence.strip() + '. '
                    test_chunk = temp_chunk + sentence

                    if count_tokens(test_chunk, model_name) <= max_tokens:
                        temp_chunk = test_chunk
                    else:
                        if temp_chunk:
                            chunks.append(temp_chunk.strip())
                        temp_chunk = sentence

                if temp_chunk.strip():
                    current_chunk = temp_chunk.strip()
            else:
                current_chunk = paragraph

    # Add the last chunk if it exists
    if current_chunk:
        chunks.append(current_chunk.strip())

    return chunks