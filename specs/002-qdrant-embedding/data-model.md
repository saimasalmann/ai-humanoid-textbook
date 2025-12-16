# Data Model: Embed Docusaurus Book Content into Qdrant

## Document Chunk Entity

**Description**: A segment of content from an MD/MDX file, approximately 348 tokens in length, containing the text content and associated metadata.

**Fields**:
- `id` (string): Unique identifier for the chunk, typically generated as UUID
- `content` (string): The actual text content of the chunk (up to ~348 tokens)
- `file_path` (string): Relative path of the source file within the /docs directory
- `chunk_index` (integer): Sequential index of the chunk within the source document
- `chapter` (string): Chapter or section title extracted from the document
- `vector` (array of floats): Embedding vector representation of the content
- `created_at` (datetime): Timestamp when the chunk was created

**Relationships**:
- Belongs to a source MD/MDX file
- Stored as a point in Qdrant collection

**Validation Rules**:
- Content must not exceed token limit (approximately 348 tokens)
- File path must be a valid MD/MDX file within /docs directory
- Chunk index must be a non-negative integer
- Vector must match the dimensionality required by the embedding model

## Qdrant Collection Schema

**Description**: Storage location for embeddings and metadata in Qdrant Cloud.

**Fields**:
- `id` (string): Point ID in Qdrant (same as Document Chunk ID)
- `vector` (array of floats): The embedding vector
- `payload` (object): Metadata object containing:
  - `content`: The original text content
  - `file_path`: Source file path
  - `chunk_index`: Position in source document
  - `chapter`: Chapter/section information
  - `created_at`: Creation timestamp

**Vector Configuration**:
- Size: Depends on embedding model (1536 for OpenAI ada-002)
- Distance: Cosine similarity

## Embedding Vector Entity

**Description**: Numerical representation of document chunk content suitable for semantic similarity search.

**Fields**:
- `vector_data` (array of floats): The actual embedding values
- `model_used` (string): Name of the model used to generate the embedding
- `dimension` (integer): Number of dimensions in the vector
- `source_chunk_id` (string): Reference to the source Document Chunk

## Processing State Entity

**Description**: Tracks the state of the embedding process for monitoring and error recovery.

**Fields**:
- `process_id` (string): Unique identifier for the processing run
- `total_files` (integer): Number of files to process
- `processed_files` (integer): Number of files processed successfully
- `total_chunks` (integer): Total number of chunks generated
- `embedded_chunks` (integer): Number of chunks successfully embedded
- `status` (string): Current status (running, completed, failed, paused)
- `start_time` (datetime): When the process started
- `end_time` (datetime): When the process completed or failed
- `errors` (array of objects): List of errors encountered during processing