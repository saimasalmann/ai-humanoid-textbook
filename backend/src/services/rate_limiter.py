import time
try:
    import redis.asyncio as redis
    REDIS_AVAILABLE = True
except ImportError:
    REDIS_AVAILABLE = False
    redis = None

from typing import Dict
from ..config.settings import settings


class RateLimiter:
    """
    Service class to handle rate limiting using Redis with token bucket algorithm.
    """
    def __init__(self):
        self.redis_client = None
        self.requests = settings.rate_limit_requests
        self.window = settings.rate_limit_window
        self._in_memory_buckets: Dict[str, list] = {}  # In-memory fallback for rate limiting

    async def init_redis(self):
        """
        Initialize Redis connection.
        """
        self.redis_client = redis.from_url(settings.redis_url)

    async def is_allowed(self, key: str) -> bool:
        """
        Check if a request is allowed based on rate limiting.

        Args:
            key: Unique identifier for the rate limit (e.g., session_id)

        Returns:
            True if request is allowed, False otherwise
        """
        if REDIS_AVAILABLE:
            if not self.redis_client:
                await self.init_redis()

            # Token bucket algorithm implementation with Redis
            current_time = time.time()
            bucket_key = f"rate_limit:{key}"

            # Get the current state of the bucket
            pipe = self.redis_client.pipeline()
            pipe.lrange(bucket_key, 0, -1)
            pipe.expire(bucket_key, self.window)
            result = await pipe.execute()

            tokens = result[0]

            # Filter tokens to only include those within the time window
            valid_tokens = [float(t) for t in tokens if current_time - float(t) < self.window]

            # If we have space for more requests, allow the request
            if len(valid_tokens) < self.requests:
                # Add a new token for the current request
                await self.redis_client.lpush(bucket_key, current_time)
                await self.redis_client.expire(bucket_key, self.window)
                return True
            else:
                # No more tokens available in the current window
                return False
        else:
            # In-memory rate limiting fallback
            current_time = time.time()
            bucket_key = key

            if bucket_key not in self._in_memory_buckets:
                self._in_memory_buckets[bucket_key] = []

            # Filter tokens to only include those within the time window
            valid_tokens = [t for t in self._in_memory_buckets[bucket_key] if current_time - t < self.window]

            # If we have space for more requests, allow the request
            if len(valid_tokens) < self.requests:
                # Add a new token for the current request
                self._in_memory_buckets[bucket_key].append(current_time)
                return True
            else:
                # No more tokens available in the current window
                return False

    async def get_reset_time(self, key: str) -> int:
        """
        Get the time when the rate limit will reset for a given key.

        Args:
            key: Unique identifier for the rate limit (e.g., session_id)

        Returns:
            Unix timestamp when the rate limit will reset
        """
        if REDIS_AVAILABLE:
            if not self.redis_client:
                await self.init_redis()

            bucket_key = f"rate_limit:{key}"
            ttl = await self.redis_client.ttl(bucket_key)
            return int(time.time() + ttl) if ttl > 0 else int(time.time())
        else:
            # For in-memory implementation, return current time + window
            # since we don't track exact TTL
            return int(time.time() + self.window)


# Global rate limiter instance
rate_limiter = RateLimiter()