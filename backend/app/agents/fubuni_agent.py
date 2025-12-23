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
OPENAI_BASE_URL = "https://openrouter.ai/api/v1"  # Fixed - never change this

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
        # Using NVIDIA Nemotron Nano 30B - free model with good tool calling support
        model_name = "nvidia/nemotron-3-nano-30b-a3b:free"
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
    didn't return enough information. Provide additional context to expand the search.

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


def _detect_chapter_from_content(content: str) -> Optional[str]:
    """Detect the appropriate chapter based on content keywords."""
    content_lower = content.lower()

    # Check for chapter-related keywords
    if any(kw in content_lower for kw in ['introduction', 'basics', 'overview', 'what is', 'humanoid robot', 'getting started']):
        return "introduction-to-humanoid-robotics"
    elif any(kw in content_lower for kw in ['sensor', 'camera', 'lidar', 'perception', 'vision', 'detection', 'sensing']):
        return "sensors-and-perception"
    elif any(kw in content_lower for kw in ['actuator', 'motor', 'servo', 'movement', 'joint', 'dof', 'locomotion']):
        return "actuators-and-movement"
    elif any(kw in content_lower for kw in ['control', 'pid', 'feedback', 'stability', 'loop', 'controller']):
        return "control-systems"
    elif any(kw in content_lower for kw in ['navigation', 'path planning', 'slam', 'trajectory', 'waypoint']):
        return "path-planning-and-navigation"
    return None


class FubuniAgent:
    def __init__(self):
        _initialize_agent()
        self.agent = Agent(
            name="Fubuni",
            instructions="""You are Fubuni, an AI assistant for Physical Humanoid Robotics documentation platform.

## WORKFLOW - FOLLOW THIS EXACTLY:
1. User asks a question
2. IMMEDIATELY call `search_knowledge_base` tool - DO NOT describe what you will do, just call the tool
3. Wait for tool results
4. Respond based on the results

## RULES:

1. **ALWAYS CALL THE TOOL FIRST**: For ANY question (except "hi", "hello", "thanks"), you MUST call `search_knowledge_base` before responding. Do not say "I'll search" - just call the tool.

2. **BE EFFICIENT**: Make only 1-2 tool calls maximum. After getting results, respond immediately.

3. **USE THE RESULTS**: Base your answer on the tool results. If results are relevant, cite them. If not found, answer from general knowledge.

4. **SIMPLE GREETINGS ONLY**: Only skip tool calls for simple greetings like "hi", "hello", "thanks".

Keep responses concise and helpful.""",
            tools=[search_knowledge_base, search_knowledge_base_detailed, get_robotics_info],
            # Note: Removed output_type to allow proper tool calling - we handle structured output manually
        )

    async def process_message(self, message: str, session_id: str = None) -> AgentResponse:
        """
        Process a user message and return the agent's structured response
        Returns AgentResponse with response text and optional chapter navigation
        """
        try:
            result = await Runner.run(
                starting_agent=self.agent,
                input=message,
                run_config=config,
                max_turns=15  # Increased from default 10
            )
            # Get the response text
            response_text = str(result.final_output) if result.final_output else ""

            # Detect chapter from the response content
            chapter = _detect_chapter_from_content(response_text)

            # Also check the original message for chapter hints
            if not chapter:
                chapter = _detect_chapter_from_content(message)

            return AgentResponse(
                response=response_text,
                chapter=chapter,
                section=None,
                should_navigate=chapter is not None
            )
        except MaxTurnsExceeded as e:
            # Gracefully handle turn limit - extract partial results from the run
            partial_response = self._extract_partial_response(e)
            if partial_response:
                chapter = _detect_chapter_from_content(partial_response)
                return AgentResponse(
                    response=f"{partial_response}\n\n---\n*Note: Response was synthesized from available search results due to processing limits.*",
                    chapter=chapter,
                    should_navigate=chapter is not None
                )
            return AgentResponse(
                response="I found some information but couldn't complete the full analysis. Please try asking a more specific question or break it down into smaller parts."
            )
        except Exception as e:
            return AgentResponse(response=f"Error processing message: {str(e)}")

    def _extract_partial_response(self, exc: MaxTurnsExceeded) -> str:
        """
        Extract any useful partial response from a MaxTurnsExceeded exception.
        The exception contains the RunResult with all the messages/tool calls made.
        """
        try:
            # The exception has a 'result' attribute with the partial RunResult
            if hasattr(exc, 'result') and exc.result:
                run_result = exc.result
                # Try to get the last assistant message content
                if hasattr(run_result, 'new_items') and run_result.new_items:
                    # Look for the last text content from assistant
                    for item in reversed(run_result.new_items):
                        if hasattr(item, 'content') and item.content:
                            # Extract text from content
                            for content_part in item.content:
                                if hasattr(content_part, 'text') and content_part.text:
                                    return content_part.text
                        # Also check for raw_item with output
                        if hasattr(item, 'raw_item'):
                            raw = item.raw_item
                            if hasattr(raw, 'content') and raw.content:
                                for part in raw.content:
                                    if hasattr(part, 'text'):
                                        return part.text
                # Try to get tool call results (RAG search results)
                if hasattr(run_result, 'new_items'):
                    tool_results = []
                    for item in run_result.new_items:
                        if hasattr(item, 'output') and item.output:
                            # This is likely a tool result
                            tool_results.append(str(item.output))
                    if tool_results:
                        # Return the last tool result as it likely has the most relevant info
                        return f"Based on the documentation search:\n\n{tool_results[-1]}"
        except Exception:
            pass
        return None

    async def process_message_streamed(self, message: str, session_id: str = None):
        """
        Process a user message and return a streaming response
        """
        try:
            result = Runner.run_streamed(
                starting_agent=self.agent,
                input=message,
                run_config=config,
                max_turns=15  # Increased from default 10
            )
            collected_content = []
            async for event in result.stream_events():
                if event.type == "raw_response_event" and isinstance(
                    event.data, ResponseTextDeltaEvent
                ):
                    collected_content.append(event.data.delta)
                    yield event.data.delta
        except MaxTurnsExceeded as e:
            # For streaming, yield a graceful message
            partial = self._extract_partial_response(e)
            if partial:
                yield f"\n\n📚 **Source: Documentation Knowledge Base**\n\n{partial}\n\n---\n*Note: Response was synthesized from available search results due to processing limits.*"
            else:
                yield "\n\nI found some information but couldn't complete the full analysis. Please try asking a more specific question."
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
