# Quickstart: Embed Docusaurus Book Content into Qdrant

## Prerequisites

- Python 3.12+
- pip package manager
- Access to Qdrant Cloud instance
- Docusaurus project with MD/MDX files in `/docs` folder

## Setup

1. **Install Dependencies**:
   ```bash
   pip install qdrant-client python-dotenv langchain tiktoken
   ```

2. **Configure Environment Variables**:
   Create a `.env` file in the project root with:
   ```env
   QDRANT_URL=your-qdrant-cluster-url
   QDRANT_API_KEY=your-api-key
   COLLECTION_NAME=your-collection-name
   ```

3. **Prepare Your Documentation**:
   Ensure your Docusaurus project has MD/MDX files in the `/docs` directory.

## Usage

1. **Run the Embedding Script**:
   ```bash
   python scripts/embed.py
   ```

2. **Monitor Progress**:
   The script will output progress information as it processes files and generates embeddings.

3. **Verify Results**:
   Check your Qdrant Cloud collection to confirm vectors and metadata have been stored correctly.

## Configuration Options

- Adjust chunk size by modifying the token limit in the script
- Change embedding model by updating the model identifier
- Modify retry settings for network operations

## Troubleshooting

- If you encounter memory issues with large files, reduce the batch size
- Ensure your QDRANT_API_KEY has appropriate permissions for the collection
- Check that the QDRANT_URL is accessible from your network