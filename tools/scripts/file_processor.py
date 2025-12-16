"""
File processing module for the Qdrant embedding pipeline.

This module handles discovery and content extraction from MD/MDX files.
"""
import os
from pathlib import Path
from typing import List, Tuple
import re


def find_md_mdx_files(docs_path: str = "./docs") -> List[str]:
    """
    Recursively find all MD/MDX files in the docs directory.

    Args:
        docs_path (str): Path to the docs directory

    Returns:
        List[str]: List of file paths to MD/MDX files
    """
    docs_dir = Path(docs_path)
    if not docs_dir.exists():
        raise FileNotFoundError(f"Docs directory does not exist: {docs_path}")

    md_mdx_files = []
    for ext in [".md", ".mdx"]:
        files = docs_dir.rglob(f"*{ext}")
        md_mdx_files.extend([str(f) for f in files if f.is_file()])

    return sorted(md_mdx_files)


def extract_content_from_file(file_path: str) -> Tuple[str, str]:
    """
    Extract content and potential chapter/section title from an MD/MDX file.

    Args:
        file_path (str): Path to the MD/MDX file

    Returns:
        Tuple[str, str]: Content of the file and extracted chapter title
    """
    with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
        content = f.read()

    # Extract the first heading as the chapter title (if available)
    chapter = extract_chapter_title(content, file_path)

    return content, chapter


def extract_chapter_title(content: str, file_path: str) -> str:
    """
    Extract the main title from the content or use the filename as fallback.

    Args:
        content (str): Content of the file
        file_path (str): Path to the file

    Returns:
        str: Extracted or derived chapter title
    """
    # Look for the first heading (h1) in the content
    h1_pattern = r'^#\s+(.+)$'
    match = re.search(h1_pattern, content, re.MULTILINE)

    if match:
        return match.group(1).strip()

    # If no h1 found, try to extract from frontmatter
    frontmatter_pattern = r'^---\s*\n(.*?)\n---\s*\n'
    frontmatter_match = re.search(frontmatter_pattern, content, re.DOTALL | re.MULTILINE)

    if frontmatter_match:
        frontmatter = frontmatter_match.group(1)
        title_match = re.search(r'^title:\s*[\'"]?(.*?)[\'"]?$', frontmatter, re.MULTILINE)
        if title_match:
            return title_match.group(1).strip()

    # Fallback to filename without extension
    return Path(file_path).stem


def is_valid_file(file_path: str) -> bool:
    """
    Check if a file is valid for processing.

    Args:
        file_path (str): Path to the file

    Returns:
        bool: True if file is valid, False otherwise
    """
    path = Path(file_path)

    # Check if file exists and is readable
    if not path.exists() or not os.access(file_path, os.R_OK):
        return False

    # Check file extension
    if path.suffix.lower() not in ['.md', '.mdx']:
        return False

    # Check file size (don't process files larger than 50MB)
    if path.stat().st_size > 50 * 1024 * 1024:  # 50MB in bytes
        return False

    return True