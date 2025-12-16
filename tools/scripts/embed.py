#!/usr/bin/env python3
"""
Embed Docusaurus Book Content into Qdrant

This script processes MD/MDX files from a Docusaurus project, generates embeddings,
and stores them in Qdrant Cloud with proper metadata.
"""
import sys
import argparse
import os

# Add parent directory to path to allow imports
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

# Import using the parent directory structure
from scripts.file_processor import find_md_mdx_files, extract_content_from_file, is_valid_file
from scripts.chunker import create_chunks_from_content
from scripts.embedding_service import EmbeddingService
from scripts.qdrant_service import QdrantService
from scripts.config import Config
from scripts.constants import DEFAULT_CHUNK_SIZE, DEFAULT_BATCH_SIZE, DEFAULT_MODEL_NAME
from scripts.models import ProcessingState
from scripts.utils import setup_logging


def main():
    """Main entry point for the embedding pipeline."""
    parser = argparse.ArgumentParser(description="Embed Docusaurus Book Content into Qdrant")
    parser.add_argument("--docs-path", default="./docs", help="Path to the Docusaurus docs directory")
    parser.add_argument("--collection-name", help="Qdrant collection name (default: from environment)")
    parser.add_argument("--chunk-size", type=int, default=DEFAULT_CHUNK_SIZE, help="Target token size for chunks (default: 348)")
    parser.add_argument("--batch-size", type=int, default=DEFAULT_BATCH_SIZE, help="Number of chunks to process in each batch (default: 10)")
    parser.add_argument("--model", default=DEFAULT_MODEL_NAME, help="Embedding model to use (default: text-embedding-ada-002)")

    args = parser.parse_args()

    # Setup logging
    logger = setup_logging("INFO")

    # Validate configuration - this will exit with code 2 if validation fails
    Config.validate_and_exit_if_invalid()

    # Update collection name if provided
    if args.collection_name:
        Config.COLLECTION_NAME = args.collection_name

    # Initialize services
    embedding_service = EmbeddingService(model_name=args.model)
    qdrant_service = QdrantService()
    logger.info("Services initialized successfully")

    # Ensure Qdrant collection exists
    if not qdrant_service.ensure_collection_exists():
        logger.error("Failed to ensure Qdrant collection exists")
        sys.exit(3)  # Connection error exit code
    logger.info(f"Qdrant collection '{Config.COLLECTION_NAME}' is ready")

    # Find MD/MDX files
    logger.info(f"Searching for MD/MDX files in {args.docs_path}")
    try:
        file_paths = find_md_mdx_files(args.docs_path)
        logger.info(f"Found {len(file_paths)} files to process")
    except FileNotFoundError as e:
        logger.error(f"Docs directory not found: {e}")
        sys.exit(1)  # General error exit code

    # Initialize processing state
    processing_state = ProcessingState(
        process_id="embed_process_1",
        total_files=len(file_paths)
    )
    logger.info(f"Starting processing of {processing_state.total_files} files")

    # Process each file
    all_chunks = []
    for i, file_path in enumerate(file_paths):
        logger.info(f"Processing file {i+1}/{len(file_paths)}: {file_path}")

        try:
            # Extract content and chapter title
            content, chapter = extract_content_from_file(file_path)

            # Create chunks from content
            chunks = create_chunks_from_content(
                content=content,
                file_path=file_path,
                chapter=chapter,
                max_tokens=args.chunk_size
            )

            all_chunks.extend(chunks)
            processing_state.processed_files += 1

            logger.info(f"Created {len(chunks)} chunks from {file_path}")

        except UnicodeDecodeError as e:
            logger.error(f"File encoding error processing {file_path}: {e}")
            processing_state.errors.append({
                "file": file_path,
                "error": str(e),
                "type": "encoding_error"
            })
            continue
        except Exception as e:
            logger.error(f"Error processing file {file_path}: {e}")
            processing_state.errors.append({
                "file": file_path,
                "error": str(e),
                "type": "processing_error"
            })
            continue

    # Process chunks with embeddings
    logger.info(f"Processing {len(all_chunks)} chunks with embeddings")
    processed_chunks = []

    # Process in batches to manage memory
    for i in range(0, len(all_chunks), args.batch_size):
        batch = all_chunks[i:i + args.batch_size]
        logger.info(f"Processing embedding batch {i//args.batch_size + 1}/{(len(all_chunks)-1)//args.batch_size + 1}")

        try:
            # Process batch of chunks
            batch_results = embedding_service.process_chunks_batch(batch)
            batch_chunks = [result[0] for result in batch_results]  # Extract the processed chunks
            processed_chunks.extend(batch_chunks)
            processing_state.embedded_chunks += len(batch_chunks)

        except Exception as e:
            logger.error(f"Error processing embedding batch: {e}")
            processing_state.errors.append({
                "batch": i // args.batch_size + 1,
                "error": str(e),
                "type": "embedding_error"
            })
            # Continue processing other batches
            continue

    # Store embeddings in Qdrant
    logger.info(f"Storing {len(processed_chunks)} embeddings in Qdrant")
    success = qdrant_service.store_embeddings_batch(processed_chunks, batch_size=args.batch_size)

    if not success:
        logger.error("Failed to store embeddings in Qdrant")
        sys.exit(1)  # General error exit code

    # Validate storage
    expected_count = len(processed_chunks)
    if qdrant_service.validate_storage(expected_count):
        logger.info(f"Successfully stored {expected_count} embeddings in Qdrant")
        print(f"Embedding pipeline completed successfully! Stored {expected_count} embeddings in Qdrant collection '{Config.COLLECTION_NAME}'")
        sys.exit(0)  # Success exit code
    else:
        logger.error("Storage validation failed - count mismatch")
        sys.exit(1)  # General error exit code


if __name__ == "__main__":
    main()