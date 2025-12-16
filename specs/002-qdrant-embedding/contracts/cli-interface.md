# Contract: Embedding Pipeline Interface

## CLI Interface

### Main Command
```
python scripts/embed.py [options]
```

### Options
- `--docs-path`: Path to the Docusaurus docs directory (default: "./docs")
- `--collection-name`: Qdrant collection name (default: from environment)
- `--chunk-size`: Target token size for chunks (default: 348)
- `--batch-size`: Number of chunks to process in each batch (default: 10)
- `--model`: Embedding model to use (default: "text-embedding-ada-002")

### Exit Codes
- `0`: Success - all documents processed and stored in Qdrant
- `1`: General error - configuration or runtime error occurred
- `2`: Environment error - required environment variables missing
- `3`: Connection error - unable to connect to Qdrant

## Input Contract

### Environment Variables
- `QDRANT_URL`: Required - URL of the Qdrant Cloud instance
- `QDRANT_API_KEY`: Required - API key for Qdrant authentication
- `COLLECTION_NAME`: Required - Name of the target collection in Qdrant

### Input Files
- MD/MDX files located in the docs directory
- Files must be readable and contain valid content
- File paths must be valid and accessible

## Output Contract

### Success Output
- Embeddings stored in Qdrant collection
- Metadata preserved and associated with vectors
- Progress logging to stdout
- Summary statistics at completion

### Error Output
- Error messages to stderr
- Appropriate exit codes
- Error details in logs

## Processing Contract

### Data Flow
1. Read MD/MDX files from docs directory
2. Parse and extract content
3. Chunk content into token-limited segments
4. Generate embeddings for each chunk
5. Store embeddings with metadata in Qdrant
6. Validate successful storage

### Performance Guarantees
- Process 100 pages in under 10 minutes
- 95% success rate for embedding operations
- Memory usage optimized for large documents