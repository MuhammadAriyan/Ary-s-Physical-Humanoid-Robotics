from agents import (
    Agent,
    Runner,
    AsyncOpenAI,
    OpenAIChatCompletionsModel,
    function_tool,
)
from agents.run import RunConfig
import os
import sys
from dotenv import load_dotenv
from openai.types.responses import ResponseTextDeltaEvent
from ..config.settings import settings
import asyncio

# Enable verbose logging for stdout
from agents import enable_verbose_stdout_logging

# Import RAG functionality (rag_retriever.py is at /app/ in Docker)
import os.path as osp
current_dir = osp.dirname(osp.abspath(__file__))
# In Docker: /app/app/agents/ -> need /app/
# Locally: backend/app/agents/ -> need backend/
app_root = osp.join(current_dir, '..', '..')  # Go up to /app or backend
sys.path.insert(0, app_root)

try:
    from rag_retriever import search_knowledge_base as rag_search_knowledge_base
except ImportError:
    # Fallback: RAG not available
    def rag_search_knowledge_base(query: str) -> str:
        return "RAG system not available. Please check configuration."

# debugging
enable_verbose_stdout_logging()  # Enable verbose logging for stdout
load_dotenv()

# Use OpenRouter API with Nova model
OPENROUTER_API_KEY = os.getenv("OPENROUTER_API_KEY")
OPENAI_BASE_URL = os.getenv("OPENAI_BASE_URL", "https://openrouter.ai/api/v1")

# Global variables for lazy initialization
external_client = None
model = None
config = None


def _initialize_agent():
    """Initialize the agent components lazily"""
    global external_client, model, config

    if external_client is not None:
        return  # Already initialized

    # Always use OpenRouter (production ready)
    if OPENROUTER_API_KEY:
        # Use OpenRouter API key
        api_key = OPENROUTER_API_KEY
        base_url = OPENAI_BASE_URL
        model_name = "nvidia/nemotron-nano-9b-v2:free"
        use_mock = False
    else:
        # No API key - error out
        raise ValueError("OPENROUTER_API_KEY environment variable is required")

    external_client = AsyncOpenAI(api_key=api_key, base_url=base_url)

    # making type for model
    model = OpenAIChatCompletionsModel(
        openai_client=external_client,
        model=model_name,
    )

    config = RunConfig(
        model=model,
        model_provider=external_client,
        tracing_disabled=True,  # Disable tracing for this run
        workflow_name="fubuni_workflow",
    )


# RAG tool for searching the knowledge base - THIS IS THE PRIMARY TOOL
@function_tool
async def search_knowledge_base(query: str) -> str:
    """
    ALWAYS use this tool FIRST for ANY question about robotics, humanoids, sensors, actuators,
    control systems, or ANY technical topic. This searches the documentation knowledge base.

    Call this tool with different query variations if the first search doesn't return good results.
    For example:
    - First try: "humanoid robot balance"
    - If not enough: "balance control system"
    - If still not enough: "stability locomotion"

    NEVER skip this tool for technical questions. ALWAYS search before answering.
    """
    result = rag_search_knowledge_base(query)
    return result


@function_tool
async def search_knowledge_base_detailed(query: str, context: str) -> str:
    """
    Use this for a MORE DETAILED search when the first search_knowledge_base call
    didn't return enough information. Provide additional context to refine the search.

    Args:
        query: The main search query
        context: Additional context or related terms to expand the search
    """
    # Combine query with context for broader search
    combined_query = f"{query} {context}"
    result = rag_search_knowledge_base(combined_query)
    return result


@function_tool
async def get_robotics_info(topic: str) -> str:
    """
    ONLY use this as a LAST RESORT after search_knowledge_base returns no results.
    This provides general robotics information not from the documentation.
    """
    return f"[General Knowledge - Not from docs] Information about {topic}: This is general robotics knowledge. For specific documentation, the knowledge base search didn't find relevant results."


class FubuniAgent:
    def __init__(self):
        _initialize_agent()
        self.agent = Agent(
            name="Fubuni",
            instructions="""You are Fubuni, an AI assistant for Physical Humanoid Robotics documentation platform.

## CRITICAL RULES - YOU MUST FOLLOW THESE:

1. **ALWAYS SEARCH FIRST**: For ANY question about robotics, humanoids, sensors, actuators, motors, control systems, balance, locomotion, or ANY technical topic - you MUST call `search_knowledge_base` BEFORE answering. NO EXCEPTIONS.

2. **MULTIPLE SEARCHES**: If the first search doesn't return good results, try AGAIN with different keywords:
   - Rephrase the query
   - Use synonyms
   - Break down complex questions into simpler searches
   - Use `search_knowledge_base_detailed` with additional context

3. **SEARCH STRATEGY**:
   - Question: "How do humanoid robots maintain balance?"
   - Search 1: "humanoid robot balance"
   - Search 2: "balance control system stability"
   - Search 3: "locomotion equilibrium"
   - THEN synthesize the results

4. **CITE SOURCES**: When you find information, ALWAYS mention it came from the documentation.

5. **LAST RESORT ONLY**: Only use `get_robotics_info` or your general knowledge if ALL searches return no relevant results. In this case, clearly state: "I couldn't find this in the documentation, but based on general knowledge..."

6. **BE THOROUGH**: Don't be lazy. Make 2-3 search calls minimum for technical questions.

7. **NEVER SKIP RAG**: Even if you think you know the answer, SEARCH FIRST. The documentation may have specific details.

Remember: You are an assistant for THIS documentation platform. Users expect answers FROM the docs, not generic AI responses.""",
            tools=[search_knowledge_base, search_knowledge_base_detailed, get_robotics_info],
        )

    async def process_message(self, message: str, session_id: str = None) -> str:
        """
        Process a user message and return the agent's response
        """
        try:
            result = await Runner.run(
                starting_agent=self.agent, input=message, run_config=config
            )
            return result.final_output
        except Exception as e:
            return f"Error processing message: {str(e)}"

    async def process_message_streamed(self, message: str, session_id: str = None):
        """
        Process a user message and return a streaming response
        """
        try:
            result = Runner.run_streamed(
                starting_agent=self.agent, input=message, run_config=config
            )
            async for event in result.stream_events():
                if event.type == "raw_response_event" and isinstance(
                    event.data, ResponseTextDeltaEvent
                ):
                    yield event.data.delta
        except Exception as e:
            yield f"Error processing message: {str(e)}"


# Global instance of Fubuni agent (lazy initialization)
fubuni_agent = None


def get_fubuni_agent():
    """Get the global Fubuni agent instance (lazy initialization)"""
    global fubuni_agent
    if fubuni_agent is None:
        fubuni_agent = FubuniAgent()
    return fubuni_agent
