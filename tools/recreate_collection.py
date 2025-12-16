#!/usr/bin/env python3
"""
Recreate Qdrant collection with correct vector size for Gemini embeddings.
"""
import sys
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Add parent directory to path to allow imports
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)

try:
    from qdrant_client import QdrantClient
    from scripts.config import Config
    from scripts.constants import VECTOR_SIZE

    # Validate configuration
    Config.validate_and_exit_if_invalid()

    # Connect to Qdrant
    client = QdrantClient(
        url=Config.QDRANT_URL,
        api_key=Config.QDRANT_API_KEY,
        prefer_grpc=False
    )

    collection_name = Config.COLLECTION_NAME
    print(f"Recreating collection: {collection_name} with vector size: {VECTOR_SIZE}")

    # Drop existing collection if it exists
    try:
        client.get_collection(collection_name)
        print(f"Collection {collection_name} exists, dropping it...")
        client.delete_collection(collection_name)
        print(f"Collection {collection_name} dropped successfully")
    except:
        print(f"Collection {collection_name} does not exist, creating new one...")

    # Create new collection with correct vector size
    from qdrant_client.http import models
    from qdrant_client.http.models import Distance, VectorParams

    client.create_collection(
        collection_name=collection_name,
        vectors_config=VectorParams(
            size=VECTOR_SIZE,  # 768 for Gemini embeddings
            distance=Distance.COSINE
        )
    )

    print(f"Collection {collection_name} created successfully with vector size {VECTOR_SIZE}")

    # Verify collection info
    collection_info = client.get_collection(collection_name)
    print(f"Verified collection exists: {collection_name}")
    print(f"Points count: {collection_info.points_count}")
    print(f"Vector size: {collection_info.config.params.vectors.size}")
    print(f"Distance: {collection_info.config.params.vectors.distance}")

except Exception as e:
    print(f"Error recreating Qdrant collection: {e}")