"""
Retry mechanism with exponential backoff for the Qdrant embedding pipeline.

This module provides utilities for handling transient failures with retry logic.
"""
import time
import random
from functools import wraps
from typing import Callable, Type, Any
from .constants import MAX_RETRIES, RETRY_DELAY_BASE


def retry_with_exponential_backoff(
    max_retries: int = MAX_RETRIES,
    base_delay: int = RETRY_DELAY_BASE,
    max_delay: int = 60,
    exceptions: tuple = (Exception,)
):
    """
    Decorator to retry a function with exponential backoff.

    Args:
        max_retries (int): Maximum number of retry attempts
        base_delay (int): Base delay in seconds for the first retry
        max_delay (int): Maximum delay in seconds between retries
        exceptions (tuple): Tuple of exception types to catch and retry on
    """
    def decorator(func: Callable) -> Callable:
        @wraps(func)
        def wrapper(*args, **kwargs) -> Any:
            last_exception = None

            for attempt in range(max_retries + 1):
                try:
                    return func(*args, **kwargs)
                except exceptions as e:
                    last_exception = e

                    if attempt == max_retries:
                        # Final attempt - raise the exception
                        raise last_exception

                    # Calculate delay with exponential backoff and jitter
                    delay = min(base_delay * (2 ** attempt), max_delay)
                    jitter = random.uniform(0, delay * 0.1)  # Add up to 10% jitter
                    total_delay = delay + jitter

                    print(f"Attempt {attempt + 1} failed: {e}. Retrying in {total_delay:.2f}s...")
                    time.sleep(total_delay)

            # This should never be reached, but included for type safety
            raise last_exception

        return wrapper
    return decorator


class RetryHandler:
    """Class-based retry handler for more complex retry scenarios."""

    def __init__(
        self,
        max_retries: int = MAX_RETRIES,
        base_delay: int = RETRY_DELAY_BASE,
        max_delay: int = 60,
        exceptions: tuple = (Exception,)
    ):
        """
        Initialize the retry handler.

        Args:
            max_retries (int): Maximum number of retry attempts
            base_delay (int): Base delay in seconds for the first retry
            max_delay (int): Maximum delay in seconds between retries
            exceptions (tuple): Tuple of exception types to catch and retry on
        """
        self.max_retries = max_retries
        self.base_delay = base_delay
        self.max_delay = max_delay
        self.exceptions = exceptions

    def execute_with_retry(self, func: Callable, *args, **kwargs) -> Any:
        """
        Execute a function with retry logic.

        Args:
            func (Callable): Function to execute
            *args: Positional arguments to pass to the function
            **kwargs: Keyword arguments to pass to the function

        Returns:
            Any: Result of the function execution

        Raises:
            Exception: If all retry attempts fail
        """
        last_exception = None

        for attempt in range(self.max_retries + 1):
            try:
                return func(*args, **kwargs)
            except self.exceptions as e:
                last_exception = e

                if attempt == self.max_retries:
                    # Final attempt - raise the exception
                    raise last_exception

                # Calculate delay with exponential backoff and jitter
                delay = min(self.base_delay * (2 ** attempt), self.max_delay)
                jitter = random.uniform(0, delay * 0.1)  # Add up to 10% jitter
                total_delay = delay + jitter

                print(f"Attempt {attempt + 1} failed: {e}. Retrying in {total_delay:.2f}s...")
                time.sleep(total_delay)

        # This should never be reached, but included for type safety
        raise last_exception

    def execute_batch_with_retry(self, func: Callable, items: list, *args, **kwargs) -> list:
        """
        Execute a function on a batch of items with retry logic for each item.

        Args:
            func (Callable): Function to execute on each item
            items (list): List of items to process
            *args: Additional positional arguments for the function
            **kwargs: Additional keyword arguments for the function

        Returns:
            list: List of results from successful executions
        """
        results = []
        for item in items:
            try:
                result = self.execute_with_retry(func, item, *args, **kwargs)
                results.append(result)
            except Exception as e:
                print(f"Failed to process item {item} after {self.max_retries} retries: {e}")
                # Continue processing other items
                continue

        return results