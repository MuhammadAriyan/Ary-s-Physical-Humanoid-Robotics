from agents import (
    Agent,
    Runner,
    AsyncOpenAI,
    OpenAIChatCompletionsModel,
    function_tool,
)
from agents.run import RunConfig
from agents.exceptions import MaxTurnsExceeded
from pydantic import BaseModel, field_validator
from typing import Optional
import os
import sys
from dotenv import load_dotenv
from openai.types.responses import ResponseTextDeltaEvent
from ..config.settings import settings
import asyncio

# T005: Valid chapters for documentation navigation
VALID_CHAPTERS = [
    "introduction-to-humanoid-robotics",
    "sensors-and-perception",
    "actuators-and-movement",
    "control-systems",
    "path-planning-and-navigation",
]

# T004: Structured response model for documentation navigation
class AgentResponse(BaseModel):
    """Structured agent response with documentation navigation hints"""
    response: str
    chapter: Optional[str] = None
    section: Optional[str] = None
    should_navigate: bool = False

    @field_validator('chapter')
    @classmethod
    def validate_chapter(cls, v):
        if v is not None and v not in VALID_CHAPTERS:
            return None  # Fallback to no navigation for invalid chapters
        return v

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


# RAG tool for searching the knowledge base
@function_tool
async def search_knowledge_base(query: str) -> str:
    """
    Search the robotics documentation knowledge base. Use this for technical questions.
    Returns relevant documentation excerpts.
    """
    result = rag_search_knowledge_base(query)
    return result


class FubuniAgent:
    def __init__(self):
        _initialize_agent()
        self.agent = Agent(
            name="Fubuni",
            instructions="""You are Fubuni, a helpful AI assistant for a Physical Humanoid Robotics documentation site.

RULES:
1. For technical questions about robotics, use search_knowledge_base ONCE to find relevant info.
2. After searching, respond with what you found. Don't search multiple times.
3. For greetings or simple questions, respond directly without searching.
4. Keep responses concise and helpful.
5. If the search doesn't return useful results, just answer based on general knowledge.

Be friendly and efficient!""",
            tools=[search_knowledge_base],
        )

    def _detect_chapter(self, message: str, response: str) -> tuple[Optional[str], bool]:
        """Detect which chapter the response relates to based on keywords."""
        text = (message + " " + response).lower()

        # Order matters - more specific chapters first
        chapter_keywords = {
            "control-systems": ["pid", "control system", "feedback loop", "controller", "control-systems"],
            "sensors-and-perception": ["sensor", "camera", "lidar", "perception", "vision", "imu", "gyroscope", "sensors-and-perception"],
            "actuators-and-movement": ["actuator", "motor", "servo", "joint", "dof", "locomotion", "actuators-and-movement"],
            "path-planning-and-navigation": ["path planning", "slam", "trajectory", "waypoint", "navigation", "path-planning"],
            "introduction-to-humanoid-robotics": ["introduction", "basics", "overview", "what is humanoid", "getting started"],
        }

        for chapter, keywords in chapter_keywords.items():
            if any(kw in text for kw in keywords):
                return chapter, True
        return None, False

    async def process_message(self, message: str, session_id: str = None) -> AgentResponse:
        """
        Process a user message and return the agent's structured response.
        """
        try:
            result = await Runner.run(
                starting_agent=self.agent,
                input=message,
                run_config=config,
                max_turns=5  # Keep it low to avoid loops
            )
            response_text = str(result.final_output)
            chapter, should_navigate = self._detect_chapter(message, response_text)
            return AgentResponse(
                response=response_text,
                chapter=chapter,
                should_navigate=should_navigate
            )
        except MaxTurnsExceeded:
            return AgentResponse(
                response="I'm having trouble processing this request. Please try a simpler question."
            )
        except Exception as e:
            return AgentResponse(response=f"Error: {str(e)}")

    async def process_message_streamed(self, message: str, session_id: str = None):
        """
        Process a user message and return a streaming response.
        """
        try:
            result = Runner.run_streamed(
                starting_agent=self.agent,
                input=message,
                run_config=config,
                max_turns=5
            )
            async for event in result.stream_events():
                if event.type == "raw_response_event" and isinstance(
                    event.data, ResponseTextDeltaEvent
                ):
                    yield event.data.delta
        except MaxTurnsExceeded:
            yield "I'm having trouble processing this. Please try a simpler question."
        except Exception as e:
            yield f"Error: {str(e)}"


# Global instance of Fubuni agent (lazy initialization)
fubuni_agent = None


def get_fubuni_agent():
    """Get the global Fubuni agent instance (lazy initialization)"""
    global fubuni_agent
    if fubuni_agent is None:
        fubuni_agent = FubuniAgent()
    return fubuni_agent
