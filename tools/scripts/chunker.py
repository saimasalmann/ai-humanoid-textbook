"""
Chunking module for the Qdrant embedding pipeline.

This module handles splitting content into ~348-token segments with metadata preservation.
"""
from typing import List, Tuple
from .models import DocumentChunk
from .utils import count_tokens, chunk_text
from .constants import DEFAULT_CHUNK_SIZE
import uuid


def create_chunks_from_content(
    content: str,
    file_path: str,
    chapter: str,
    max_tokens: int = DEFAULT_CHUNK_SIZE
) -> List[DocumentChunk]:
    """
    Split content into ~348-token segments with preserved metadata.

    Args:
        content (str): The content to chunk
        file_path (str): Path to the source file
        chapter (str): Chapter or section title
        max_tokens (int): Maximum number of tokens per chunk

    Returns:
        List[DocumentChunk]: List of DocumentChunk objects with content and metadata
    """
    # Split the content into chunks that don't exceed the token limit
    text_chunks = chunk_text(content, max_tokens=max_tokens)

    # Create DocumentChunk objects with metadata
    chunks = []
    for i, text in enumerate(text_chunks):
        chunk = DocumentChunk(
            id=str(uuid.uuid4()),
            content=text,
            file_path=file_path,
            chunk_index=i,
            chapter=chapter
        )
        chunks.append(chunk)

    return chunks


def validate_chunk_size(chunk: DocumentChunk, max_tokens: int = DEFAULT_CHUNK_SIZE) -> bool:
    """
    Validate that a chunk doesn't exceed the token limit.

    Args:
        chunk (DocumentChunk): The chunk to validate
        max_tokens (int): Maximum allowed tokens

    Returns:
        bool: True if chunk is valid, False otherwise
    """
    token_count = count_tokens(chunk.content)
    return token_count <= max_tokens


def merge_small_chunks(
    chunks: List[DocumentChunk],
    max_tokens: int = DEFAULT_CHUNK_SIZE
) -> List[DocumentChunk]:
    """
    Merge small chunks together if they're below a certain threshold.

    Args:
        chunks (List[DocumentChunk]): List of chunks to potentially merge
        max_tokens (int): Maximum tokens allowed per chunk

    Returns:
        List[DocumentChunk]: List of merged chunks
    """
    if not chunks:
        return chunks

    merged_chunks = []
    current_chunk = chunks[0]

    for next_chunk in chunks[1:]:
        # Check if combining the current chunk with the next chunk would exceed the limit
        combined_content = current_chunk.content + "\n\n" + next_chunk.content
        combined_token_count = count_tokens(combined_content)

        if combined_token_count <= max_tokens:
            # Merge the chunks
            current_chunk = DocumentChunk(
                id=current_chunk.id,  # Keep the first chunk's ID
                content=combined_content,
                file_path=current_chunk.file_path,
                chunk_index=current_chunk.chunk_index,  # Keep the first chunk's index
                chapter=current_chunk.chapter,
                vector=current_chunk.vector,
                created_at=current_chunk.created_at
            )
        else:
            # Add the current chunk to the result and start a new one
            merged_chunks.append(current_chunk)
            current_chunk = next_chunk

    # Add the last chunk
    merged_chunks.append(current_chunk)

    return merged_chunks