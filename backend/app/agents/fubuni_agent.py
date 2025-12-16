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
        model_name = "amazon/nova-2-lite-v1:free"
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


@function_tool
async def get_robotics_info(topic: str) -> str:
    """Fetch information about robotics topics. Useful for answering questions about humanoid robots, actuators, sensors, etc."""
    return f"Here's some information about {topic}: This is a robotics documentation platform. You can find detailed information about various robotics topics in the documentation."


# RAG tool for searching the knowledge base
@function_tool
async def search_knowledge_base(query: str) -> str:
    """Search private documents for factual information. Use only when user asks about specific topics likely covered in docs."""
    # Call the RAG function from rag_retriever module
    result = rag_search_knowledge_base(query)
    return result

# Tool to determine if a query should use RAG
@function_tool
async def should_use_rag(query: str) -> str:
    """Determine if the user's query is likely to require factual/technical information from the knowledge base."""
    # Simple heuristic: check if query contains keywords suggesting factual/technical nature
    query_lower = query.lower()
    technical_keywords = [
        'what is', 'how does', 'explain', 'describe', 'define', 'specification',
        'procedure', 'process', 'method', 'technique', 'component', 'system',
        'architecture', 'design', 'principle', 'mechanism', 'algorithm', 'formula',
        'data', 'statistics', 'research', 'study', 'findings', 'results'
    ]

    # Check if query contains technical keywords
    for keyword in technical_keywords:
        if keyword in query_lower:
            return "yes"

    # Check if query asks about specific documented topics
    documented_topics = [
        'robot', 'humanoid', 'actuator', 'sensor', 'balance', 'locomotion',
        'control system', 'programming', 'calibration', 'safety', 'protocol',
        'spec', 'documentation', 'manual', 'guide', 'handbook', 'tutorial'
    ]

    for topic in documented_topics:
        if topic in query_lower:
            return "yes"

    # If no strong indicators, default to not using RAG
    return "no"


class FubuniAgent:
    def __init__(self):
        _initialize_agent()
        self.agent = Agent(
            name="Fubuni",
            instructions="You are Fubuni, an AI assistant for Physical Humanoid Robotics documentation platform. Always respond helpfully and accurately. If you don't know something, respond with 'I'm not sure yet, teach me!'. Be concise but informative in your responses.\n\n"
                         "When a user asks a question, first consider if it's likely to require factual or technical information from the documentation. "
                         "Use the should_use_rag tool to help determine this. If the answer is 'yes', use the search_knowledge_base tool to retrieve "
                         "relevant information from the documentation. Always prioritize information from the knowledge base for factual topics. "
                         "If no relevant documents are found, fall back to your general knowledge but clearly indicate that you're not referencing the documentation. "
                         "Use the get_robotics_info tool when users ask about general robotics topics that might not be in the specific documentation.\n\n"
                         "Always cite sources when using information from the knowledge base.",
            tools=[get_robotics_info, search_knowledge_base, should_use_rag],
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
