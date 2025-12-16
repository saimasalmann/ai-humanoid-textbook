#!/usr/bin/env python3
"""
Check Qdrant collection status to see if embeddings have been completed.
"""
import os
import sys
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
    print(f"Checking collection: {collection_name}")

    # Get collection info
    try:
        collection_info = client.get_collection(collection_name)
        print(f"Collection exists: {collection_name}")
        print(f"Points count: {collection_info.points_count}")

        # Access vector configuration properly
        if hasattr(collection_info.config.params, 'vectors'):
            vectors_config = collection_info.config.params.vectors
            if hasattr(vectors_config, 'size'):
                print(f"Vector size: {vectors_config.size}")
            if hasattr(vectors_config, 'distance'):
                print(f"Distance: {vectors_config.distance}")
        elif hasattr(collection_info.config.params, 'vector'):
            # Handle different Qdrant client versions
            vector_config = collection_info.config.params.vector
            if hasattr(vector_config, 'size'):
                print(f"Vector size: {vector_config.size}")
            if hasattr(vector_config, 'distance'):
                print(f"Distance: {vector_config.distance}")

        # Check a few points if any exist
        if collection_info.points_count > 0:
            print("\nSample points:")
            try:
                points = client.scroll(
                    collection_name=collection_name,
                    limit=2,
                    with_payload=True,
                    with_vectors=False
                )
                # Handle different return formats
                if isinstance(points, tuple) and len(points) == 2:
                    records, _ = points
                    for i, record in enumerate(records):
                        print(f"  Point {i+1}: ID={record.id}")
                        print(f"    Payload keys: {list(record.payload.keys()) if record.payload else 'None'}")
                        if record.payload:
                            content_preview = str(record.payload.get('content', ''))[:100]
                            print(f"    Content preview: {content_preview}...")
                else:
                    # Handle if points is a list directly
                    for i, record in enumerate(points[:2]):
                        print(f"  Point {i+1}: ID={record.id}")
                        print(f"    Payload keys: {list(record.payload.keys()) if record.payload else 'None'}")
                        if record.payload:
                            content_preview = str(record.payload.get('content', ''))[:100]
                            print(f"    Content preview: {content_preview}...")
            except Exception as e:
                print(f"  Could not retrieve sample points: {e}")
        else:
            print("No points found in collection.")

    except Exception as e:
        print(f"Collection '{collection_name}' does not exist or is not accessible: {e}")

except ImportError as e:
    print(f"Missing dependencies. Install requirements first: {e}")
    print("Run: pip install -r requirements.txt")
except Exception as e:
    print(f"Error checking Qdrant: {e}")