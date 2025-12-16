try:
    import asyncpg
    ASYNCPG_AVAILABLE = True
except ImportError:
    ASYNCPG_AVAILABLE = False
    asyncpg = None

from typing import Optional, List
from contextlib import asynccontextmanager
from datetime import datetime
from ..config.settings import settings
from ..models.chat_session import ChatSession
from ..models.message import Message
from ..models.query_request import QueryRequest


class DatabaseService:
    """
    Service class to handle database operations for Neon Postgres.
    """
    def __init__(self):
        self.pool: Optional[asyncpg.Pool] = None
        # In-memory storage for development when asyncpg is not available
        self._sessions: dict = {}
        self._messages: dict = {}

    async def connect(self):
        """
        Initialize the database connection pool.
        """
        if not ASYNCPG_AVAILABLE:
            print("asyncpg not available, skipping database initialization")
            return

        if settings.neon_database_url:
            self.pool = await asyncpg.create_pool(
                dsn=settings.neon_database_url,
                min_size=1,
                max_size=10,
                command_timeout=60
            )
        else:
            # For development without database
            print("No database URL provided, skipping database initialization")

    async def disconnect(self):
        """
        Close the database connection pool.
        """
        if self.pool:
            await self.pool.close()

    def _has_asyncpg_connection(self):
        """
        Check if asyncpg is available and connected.
        """
        return ASYNCPG_AVAILABLE and self.pool is not None

    async def _execute_query_with_asyncpg(self, query, *args):
        """
        Execute a query using asyncpg if available.
        """
        if not self._has_asyncpg_connection():
            raise Exception("Database not initialized - asyncpg not available or not connected")

        async with self.pool.acquire() as conn:
            return await conn.fetch(query, *args)

    async def _execute_command_with_asyncpg(self, command, *args):
        """
        Execute a command using asyncpg if available.
        """
        if not self._has_asyncpg_connection():
            raise Exception("Database not initialized - asyncpg not available or not connected")

        async with self.pool.acquire() as conn:
            return await conn.execute(command, *args)

    async def create_session(self, session: ChatSession):
        """
        Create a new chat session in the database.
        """
        if self._has_asyncpg_connection():
            await self._execute_command_with_asyncpg(
                """
                INSERT INTO chat_sessions (session_id, user_id, created_at, last_active_at, expires_at)
                VALUES ($1, $2, $3, $4, $5)
                """,
                session.session_id,
                session.user_id,
                session.created_at,
                session.last_active_at,
                session.expires_at
            )
        else:
            # Store in memory for development
            self._sessions[session.session_id] = session

    async def get_session(self, session_id: str) -> Optional[ChatSession]:
        """
        Get a chat session by ID from the database.
        """
        if self._has_asyncpg_connection():
            try:
                rows = await self._execute_query_with_asyncpg(
                    """
                    SELECT session_id, user_id, created_at, last_active_at, expires_at
                    FROM chat_sessions
                    WHERE session_id = $1
                    """,
                    session_id
                )
                if rows:
                    row = rows[0]
                    return ChatSession(
                        session_id=row['session_id'],
                        user_id=row['user_id'],
                        created_at=row['created_at'],
                        last_active_at=row['last_active_at'],
                        expires_at=row['expires_at']
                    )
                return None
            except Exception:
                # Fallback to in-memory storage if query fails
                return self._sessions.get(session_id)
        else:
            # Use in-memory storage
            return self._sessions.get(session_id)

    async def update_session_activity(self, session_id: str, last_active_at: datetime):
        """
        Update the last active time for a session.
        """
        if self._has_asyncpg_connection():
            await self._execute_command_with_asyncpg(
                """
                UPDATE chat_sessions
                SET last_active_at = $1
                WHERE session_id = $2
                """,
                last_active_at,
                session_id
            )
        else:
            # Update in-memory storage
            if session_id in self._sessions:
                session = self._sessions[session_id]
                session.last_active_at = last_active_at
                self._sessions[session_id] = session

    async def create_message(self, message: Message):
        """
        Create a new message in the database.
        """
        if self._has_asyncpg_connection():
            await self._execute_command_with_asyncpg(
                """
                INSERT INTO messages (message_id, session_id, role, content, timestamp, query_mode, selected_text)
                VALUES ($1, $2, $3, $4, $5, $6, $7)
                """,
                message.message_id,
                message.session_id,
                message.role,
                message.content,
                message.timestamp,
                message.query_mode,
                message.selected_text
            )
        else:
            # Store in memory for development
            if message.session_id not in self._messages:
                self._messages[message.session_id] = []
            self._messages[message.session_id].append(message)

    async def get_messages_by_session(self, session_id: str, limit: int = 50) -> List[Message]:
        """
        Get messages for a specific session.
        """
        if self._has_asyncpg_connection():
            try:
                rows = await self._execute_query_with_asyncpg(
                    """
                    SELECT message_id, session_id, role, content, timestamp, query_mode, selected_text
                    FROM messages
                    WHERE session_id = $1
                    ORDER BY timestamp DESC
                    LIMIT $2
                    """,
                    session_id,
                    limit
                )
                messages = []
                for row in rows:
                    message = Message(
                        message_id=row['message_id'],
                        session_id=row['session_id'],
                        role=row['role'],
                        content=row['content'],
                        timestamp=row['timestamp'],
                        query_mode=row['query_mode'],
                        selected_text=row['selected_text']
                    )
                    messages.append(message)
                return messages
            except Exception:
                # Fallback to in-memory storage if query fails
                session_messages = self._messages.get(session_id, [])
                # Sort by timestamp descending and limit
                sorted_messages = sorted(session_messages, key=lambda m: m.timestamp, reverse=True)
                return sorted_messages[:limit]
        else:
            # Use in-memory storage
            session_messages = self._messages.get(session_id, [])
            # Sort by timestamp descending and limit
            sorted_messages = sorted(session_messages, key=lambda m: m.timestamp, reverse=True)
            return sorted_messages[:limit]

    async def create_query_request(self, query_request: QueryRequest):
        """
        Create a new query request in the database.
        """
        # For now, just log this since we don't have a query_requests table defined
        # In a real implementation, you would create and insert into a query_requests table
        print(f"Query request created: {query_request.request_id} for session {query_request.session_id}")

    async def delete_expired_sessions(self):
        """
        Delete expired sessions from the database.
        """
        if self._has_asyncpg_connection():
            await self._execute_command_with_asyncpg(
                """
                DELETE FROM chat_sessions
                WHERE expires_at < NOW()
                """
            )
            await self._execute_command_with_asyncpg(
                """
                DELETE FROM messages
                WHERE session_id NOT IN (SELECT session_id FROM chat_sessions)
                """
            )
        else:
            # Clean up expired sessions in memory
            now = datetime.utcnow()
            expired_sessions = []
            for session_id, session in self._sessions.items():
                if session.expires_at < now:
                    expired_sessions.append(session_id)

            for session_id in expired_sessions:
                del self._sessions[session_id]
                # Also remove associated messages
                if session_id in self._messages:
                    del self._messages[session_id]


# Global database instance
db = DatabaseService()


# Example table creation queries (would be run during initialization)
CREATE_SESSIONS_TABLE = """
CREATE TABLE IF NOT EXISTS chat_sessions (
    session_id VARCHAR(255) PRIMARY KEY,
    user_id VARCHAR(255),
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    last_active_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    expires_at TIMESTAMP WITH TIME ZONE
);
"""

CREATE_MESSAGES_TABLE = """
CREATE TABLE IF NOT EXISTS messages (
    message_id VARCHAR(255) PRIMARY KEY,
    session_id VARCHAR(255) REFERENCES chat_sessions(session_id),
    role VARCHAR(20) NOT NULL,
    content TEXT NOT NULL,
    timestamp TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    query_mode VARCHAR(20) DEFAULT 'full-book',
    selected_text TEXT
);
"""