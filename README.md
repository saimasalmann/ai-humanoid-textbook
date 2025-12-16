# AI Humanoid Textbook - RAG Chatbot

This project implements a Retrieval-Augmented Generation (RAG) chatbot system with separated frontend and backend components. The system extracts content from Docusaurus documentation, generates embeddings, and provides an AI-powered chat interface.

## Project Structure

```
ai-humanoid-textbook/
├── backend/                 # FastAPI backend with RAG capabilities
│   ├── src/
│   ├── requirements.txt     # Uses uv package manager
│   └── tests/
├── frontend/                # Docusaurus-based frontend
│   ├── src/
│   ├── public/
│   ├── package.json
│   └── docusaurus.config.js
├── scripts/                 # Utility scripts
├── specs/                   # Project specifications
└── pyproject.toml           # Root project configuration for uv
```

## Features

- **Automated Content Indexing**: Recursively discovers all MD/MDX files in the docs directory
- **Smart Chunking**: Splits content into ~348-token segments with preserved metadata
- **Embedding Generation**: Uses OpenAI's text-embedding-ada-002 model (or alternatives)
- **Qdrant Integration**: Stores embeddings with metadata in Qdrant Cloud
- **Secure Configuration**: Loads all sensitive data from environment variables
- **Robust Error Handling**: Implements retry logic and graceful error handling
- **Progress Tracking**: Provides detailed logging and progress updates

## Prerequisites

- Python 3.12+
- pip package manager
- Access to Qdrant Cloud instance
- Docusaurus project with MD/MDX files in `/docs` folder

## Setup

1. **Install Dependencies**:

   For backend (using uv package manager):
   ```bash
   cd backend
   uv venv  # Create virtual environment
   source .venv/bin/activate  # Activate virtual environment
   uv pip install -r requirements.txt  # Install dependencies
   ```

   For frontend:
   ```bash
   cd frontend
   npm install
   ```

2. **Configure Environment Variables**:
   Copy the `.env` template and set your values:
   ```bash
   cp .env .env.example
   ```

   Edit `.env` with your configuration:
   ```env
   QDRANT_URL=your-qdrant-cluster-url
   QDRANT_API_KEY=your-api-key
   COLLECTION_NAME=your-collection-name
   OPENAI_API_KEY=your-openai-api-key  # For RAG agent
   GOOGLE_API_KEY=your-google-api-key  # For Google integration
   ```

3. **Prepare Your Documentation**:
   Ensure your Docusaurus project has MD/MDX files in the `frontend/docs` directory.

## Usage

### Backend (RAG API Server)
```bash
cd backend
source .venv/bin/activate  # If using virtual environment
uvicorn src.main:app --reload
```

### Frontend (Docusaurus Documentation)
```bash
cd frontend
npm start
```

### Embedding Pipeline (for RAG indexing)
```bash
cd backend
source .venv/bin/activate
python scripts/embed.py
```

### Command Line Options for Embedding
```bash
cd backend
python scripts/embed.py --help
```

Available options:
- `--docs-path`: Path to the Docusaurus docs directory (default: "../frontend/docs")
- `--collection-name`: Qdrant collection name (default: from environment)
- `--chunk-size`: Target token size for chunks (default: 348)
- `--batch-size`: Number of chunks to process in each batch (default: 10)
- `--model`: Embedding model to use (default: "text-embedding-ada-002")

### Example Usage
```bash
# Process docs with custom settings
cd backend
python scripts/embed.py --docs-path ../frontend/docs --chunk-size 500 --batch-size 20

# Use specific collection name
python scripts/embed.py --collection-name my-custom-collection
```

## Exit Codes

- `0`: Success - all documents processed and stored in Qdrant
- `1`: General error - configuration or runtime error occurred
- `2`: Environment error - required environment variables missing
- `3`: Connection error - unable to connect to Qdrant

## Architecture

The pipeline consists of several key components:

1. **File Processor**: Discovers and extracts content from MD/MDX files
2. **Chunker**: Splits content into appropriately sized segments
3. **Embedding Service**: Generates embeddings using AI models
4. **Qdrant Service**: Stores embeddings with metadata in Qdrant
5. **Retry Handler**: Provides exponential backoff for transient failures
6. **Configuration Manager**: Handles environment variable loading

## Performance

The system is designed to:
- Process 100 pages of documentation in under 10 minutes
- Achieve 95% successful embedding rate
- Handle large files without memory issues
- Process content in configurable batch sizes

## Testing

Run the unit tests:
```bash
pytest tests/unit/
```

Run the integration tests:
```bash
pytest tests/integration/
```

Run all tests:
```bash
pytest tests/
```

## Security

- All sensitive configuration is loaded from environment variables
- No hardcoded credentials in the codebase
- API keys are not logged or exposed

## Troubleshooting

- If you encounter memory issues with large files, reduce the batch size
- Ensure your QDRANT_API_KEY has appropriate permissions for the collection
- Check that the QDRANT_URL is accessible from your network
- Verify that your embedding model API key has sufficient quota

## Development

The project follows the Spec-Kit Plus methodology with clear separation of concerns:
- Specifications in `/specs/`
- Implementation in `/scripts/`
- Tests in `/tests/`
- Configuration in environment variables
