import os
from agents import Agent, Runner, OpenAIChatCompletionsModel, set_tracing_disabled
from openai import AsyncOpenAI
from typing import List
from ..models.retrieved_context import RetrievedContext
from ..config.settings import settings


class RAGAgent:
    """
    Custom agent implementation using OpenAI Agents SDK patterns but using Google's Gemini model.
    This agent retrieves relevant content and generates responses grounded in the textbook content.
    """
    def __init__(self):
        # Disable tracing as per your example
        set_tracing_disabled(disabled=True)

        # Set up the OpenAI-compatible client for Google's Gemini
        gemini_api_key = settings.google_api_key
        google_base_url = "https://generativelanguage.googleapis.com/v1beta/"

        if not gemini_api_key:
            print("WARNING: Google API key not set. RAG agent will not function properly.")
            self.gemini_api_key = None
            self.agent = None
            return

        external_client = AsyncOpenAI(
            api_key=gemini_api_key,
            base_url=google_base_url,
        )

        model = OpenAIChatCompletionsModel(
            model="gemini-2.0-flash",
            openai_client=external_client,
        )

        # Create the agent with instructions to prevent hallucinations
        self.agent = Agent(
            name="TextbookAssistant",
            instructions="""You are an AI assistant helping students with their textbook.
            You must answer the question based ONLY on the provided context from the textbook.
            Do not use any external knowledge or make up information.
            If the context doesn't contain enough information to answer the question,
            clearly state that the information is not available in the provided text.""",
            model=model,
        )

    async def generate_response(
        self,
        query: str,
        retrieved_contexts: List[RetrievedContext],
        query_mode: str = "full-book",
        selected_text: str = None
    ) -> str:
        """
        Generate a response based on the user query and retrieved contexts.

        Args:
            query: The user's question
            retrieved_contexts: List of relevant contexts retrieved from Qdrant
            query_mode: Either "full-book" or "selected-text"
            selected_text: The selected text (for selected-text mode)

        Returns:
            Generated response string
        """
        # Check if agent is properly initialized
        if not self.agent or not self.gemini_api_key:
            return "The AI service is not properly configured. Please contact the administrator."

        try:
            # Prepare the context content
            context_texts = []
            for ctx in retrieved_contexts:
                context_texts.append(f"Content: {ctx.content}")
                if ctx.metadata:
                    context_texts.append(f"Metadata: {ctx.metadata}")

            # Combine all context
            combined_context = "\n\n".join(context_texts)

            # Create the prompt with the retrieved context
            if query_mode == "selected-text":
                full_query = f"""
                Selected Text Context:
                {combined_context}

                Question: {query}

                Answer the question based only on the provided context.
                If the context doesn't contain enough information to answer the question,
                clearly state that the information is not available in the provided text.
                """
            else:
                full_query = f"""
                Textbook Context:
                {combined_context}

                Question: {query}

                Answer the question based only on the provided context.
                If the context doesn't contain enough information to answer the question,
                clearly state that the information is not available in the provided text.
                """

            # Use the Runner to run the agent with the query
            res = Runner.run_sync(self.agent, full_query)
            return res.final_output

        except Exception as e:
            print(f"Error generating response: {e}")
            return "Sorry, I encountered an error while processing your request. Please try again."

    async def validate_response_context(self, response: str, contexts: List[RetrievedContext]) -> bool:
        """
        Validate that the response is grounded in the provided contexts.
        This is a basic implementation - in a full system, this would be more sophisticated.

        Args:
            response: The generated response
            contexts: The contexts used to generate the response

        Returns:
            True if response appears to be grounded in contexts, False otherwise
        """
        # Basic check: see if key terms from contexts appear in response
        response_lower = response.lower()

        # Extract key terms from contexts
        key_terms = set()
        for ctx in contexts:
            content_lower = ctx.content.lower()
            # Simple approach: look for longer words that are likely to be key terms
            words = content_lower.split()
            key_terms.update([word for word in words if len(word) > 5])

        # Check if any key terms appear in response
        if not key_terms:
            return True  # If no key terms, can't validate, so assume OK

        matching_terms = sum(1 for term in key_terms if term in response_lower)
        match_ratio = matching_terms / len(key_terms)

        # Consider validated if at least 20% of key terms appear in response
        return match_ratio >= 0.2

    async def health_check(self) -> bool:
        """
        Check if the RAG agent can connect to the AI service.

        Returns:
            True if the service is accessible, False otherwise
        """
        # If the agent isn't initialized due to missing API key, it's not healthy
        if not self.agent or not self.gemini_api_key:
            return False

        try:
            # Test with a simple query to check if the AI service is working
            test_contexts = []
            test_response = await self.generate_response(
                query="Hello, can you confirm you're working?",
                retrieved_contexts=test_contexts,
                query_mode="full-book"
            )
            return len(test_response) > 0 and "not properly configured" not in test_response
        except Exception as e:
            print(f"Health check failed: {e}")
            return False


# Global RAG agent instance
rag_agent = RAGAgent()