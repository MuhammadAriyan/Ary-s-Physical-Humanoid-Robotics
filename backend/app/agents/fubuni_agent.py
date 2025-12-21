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
from typing import Optional, List
from urllib.parse import urlparse
import os
import sys
import logging
from dotenv import load_dotenv
from openai.types.responses import ResponseTextDeltaEvent
from ..config.settings import settings
import asyncio

# Import Tavily with graceful fallback
try:
    from tavily import TavilyClient
    TAVILY_AVAILABLE = True
except ImportError:
    TavilyClient = None
    TAVILY_AVAILABLE = False
    logging.warning("tavily-python not installed. Web search will be unavailable.")

# T005: Valid chapters for documentation navigation
VALID_CHAPTERS = [
    "introduction-to-humanoid-robotics",
    "sensors-and-perception",
    "actuators-and-movement",
    "control-systems",
    "path-planning-and-navigation",
]

# WebSearchResult for storing web search results
class WebSearchResult(BaseModel):
    """Single web search result from Tavily API"""
    title: str
    url: str
    snippet: str
    favicon: Optional[str] = None  # Google favicon service URL


# Global to store last web search results (cleared after each response)
_last_web_search_results: List[WebSearchResult] = []


# T004: Structured response model for documentation navigation
class AgentResponse(BaseModel):
    """Structured agent response with documentation navigation and web search hints"""
    response: str
    chapter: Optional[str] = None
    section: Optional[str] = None
    should_navigate: bool = False
    # Web search fields
    web_sources: Optional[List[WebSearchResult]] = None
    used_web_search: bool = False

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
async def search_web(query: str) -> str:
    """
    Search the web using Tavily API for robotics information NOT found in documentation.

    ONLY use this tool if:
    1. search_knowledge_base returned no relevant results or insufficient information
    2. The user's message does NOT contain "use rag" (case-insensitive)

    Returns web search results with source URLs for verification.
    The user will see these sources in a dedicated panel.

    IMPORTANT: After receiving search results, you MUST read and synthesize the information
    to provide a helpful, comprehensive answer. Do NOT just list the results.
    """
    global _last_web_search_results

    # Check if Tavily is available
    if not TAVILY_AVAILABLE:
        _last_web_search_results = []
        return "Web search unavailable - tavily-python not installed. Falling back to general knowledge."

    tavily_key = os.getenv("TAVILY_API_KEY")
    if not tavily_key:
        _last_web_search_results = []
        return "Web search unavailable - TAVILY_API_KEY not configured. Falling back to general knowledge."

    try:
        client = TavilyClient(api_key=tavily_key)
        # Search with include_images for related images
        response = client.search(
            query=query,
            max_results=10,
            include_images=True,
            include_answer=True,  # Get AI-synthesized answer
        )

        results = response.get("results", [])
        if not results:
            _last_web_search_results = []
            return "No web search results found for this query. Try using get_robotics_info for general knowledge."

        # Store structured results for the frontend panel
        _last_web_search_results = []
        for r in results:
            url = r.get("url", "")
            try:
                domain = urlparse(url).netloc if url else ""
                favicon_url = f"https://www.google.com/s2/favicons?domain={domain}&sz=32" if domain else None
            except Exception:
                favicon_url = None

            _last_web_search_results.append(WebSearchResult(
                title=r.get("title", "No title")[:200],
                url=url,
                snippet=r.get("content", "")[:500],
                favicon=favicon_url
            ))

        # Build comprehensive response for the agent to synthesize
        output_parts = ["🌐 **Web Search Results:**\n"]

        # Include Tavily's AI-generated answer if available
        if response.get("answer"):
            output_parts.append(f"**Summary:** {response['answer']}\n")

        # Include search results
        output_parts.append("\n**Sources:**\n")
        for i, r in enumerate(_last_web_search_results[:5], 1):
            output_parts.append(f"{i}. **{r.title}**\n   {r.snippet}\n   Source: {r.url}\n")

        # Include images if available (as markdown)
        images = response.get("images", [])
        if images:
            output_parts.append("\n**Related Images:**\n")
            for img_url in images[:3]:  # Limit to 3 images
                output_parts.append(f"![Related Image]({img_url})\n")

        return "\n".join(output_parts)
    except Exception as e:
        logging.warning(f"Tavily search failed: {e}")
        _last_web_search_results = []
        return f"Web search temporarily unavailable ({str(e)[:50]}). Please try the documentation search or general knowledge."


@function_tool
async def get_robotics_info(topic: str) -> str:
    """
    ONLY use this as a LAST RESORT after both search_knowledge_base AND search_web return no results.
    This provides general robotics information not from the documentation or web.
    """
    return f"[General Knowledge - Not from docs] Information about {topic}: This is general robotics knowledge. For specific documentation, the knowledge base search didn't find relevant results."


class FubuniAgent:
    def __init__(self):
        _initialize_agent()
        self.agent = Agent(
            name="Fubuni",
            instructions="""You are Fubuni, an AI assistant for Physical Humanoid Robotics documentation platform.

## RESPONSE FORMAT - SOURCE INDICATORS:
- For documentation results: Start with "📚 **Source: Documentation Knowledge Base**\n\n"
- For web search results: Start with "🌐 **Source: Web Search**\n\n"
This tells users where the information comes from.

## TOOL PRIORITY ORDER (CRITICAL):
1. **search_knowledge_base** - ALWAYS try this FIRST for any technical question
2. **search_knowledge_base_detailed** - Use if first search needs more context
3. **search_web** - ONLY use if documentation search returns no useful results
4. **get_robotics_info** - LAST RESORT for general knowledge

## CRITICAL RULES:

1. **DOCUMENTATION FIRST**: For ANY technical question - call `search_knowledge_base` FIRST.

2. **WEB SEARCH RULES**:
   - ONLY use `search_web` if documentation search returns NO useful results
   - NEVER use `search_web` if the user's message contains "use rag" (case-insensitive)
   - **FORCE WEB SEARCH**: If the user explicitly asks to "websearch", "search the web", "search online", "search browser", "look it up online", "google it", or similar phrases - SKIP documentation and use `search_web` DIRECTLY
   - **YOU MUST CALL THE TOOL** - NEVER generate fake URLs or example.com links. ALWAYS call the actual search_web tool.
   - When using web search, set `used_web_search=true` in your response

3. **SYNTHESIZE SEARCH RESULTS (CRITICAL)**:
   - After calling `search_web`, READ the returned content carefully
   - SYNTHESIZE the information into a helpful, comprehensive answer
   - DO NOT just list URLs or say "I found results" - actually explain what you found
   - Include relevant images in your response using markdown: ![description](url)
   - Cite sources naturally in your response

4. **BE EFFICIENT**: Make only 1-2 documentation searches. If still no results, try ONE web search.

5. **SEARCH STRATEGY**:
   - Search 1: Documentation with main keywords
   - Search 2 (if needed): Documentation with alternative terms
   - Search 3 (only if docs failed): Web search
   - THEN synthesize and respond with actual information

6. **CITE PROPERLY**:
   - Documentation: "According to the documentation..."
   - Web search: Include actual facts from sources, cite URLs

7. **SIMPLE QUESTIONS**: For greetings or simple questions, respond directly without searching.

## DOCUMENTATION NAVIGATION (ONLY for documentation results):
When your response is based on DOCUMENTATION (not web search), set the chapter field:

CHAPTER MAPPINGS:
- "introduction-to-humanoid-robotics" → basics, overview, what is humanoid robot, getting started
- "sensors-and-perception" → sensors, cameras, lidar, perception, vision, detection, sensing
- "actuators-and-movement" → actuators, motors, servos, movement, joints, DOF, locomotion
- "control-systems" → control, PID, feedback, stability, loops, controllers
- "path-planning-and-navigation" → navigation, path planning, SLAM, trajectory, waypoints

**IMPORTANT NAVIGATION RULES:**
- Set should_navigate=true ONLY when citing DOCUMENTATION results
- Set chapter=null for general greetings or questions not related to a specific chapter
- **NEVER set should_navigate=true or chapter when using web search** - web sources go to a separate panel
- When using web search, ALWAYS set: should_navigate=false, chapter=null

Remember: Documentation is your PRIMARY source. Web search is a FALLBACK for topics not in docs.""",
            tools=[search_knowledge_base, search_knowledge_base_detailed, search_web, get_robotics_info],
            output_type=AgentResponse,  # T006: Structured output
        )

    def _inject_web_sources(self, response: AgentResponse) -> AgentResponse:
        """
        Inject web search results from the global variable into the response.
        Clears the global variable after use.
        """
        global _last_web_search_results
        if _last_web_search_results:
            response.web_sources = _last_web_search_results.copy()
            response.used_web_search = True
            # When using web search, don't navigate to docs
            if response.used_web_search:
                response.should_navigate = False
                response.chapter = None
            _last_web_search_results = []  # Clear after use
        return response

    async def process_message(self, message: str, session_id: str = None) -> AgentResponse:
        """
        Process a user message and return the agent's structured response
        Returns AgentResponse with response text and optional chapter navigation
        """
        global _last_web_search_results
        # Clear any stale results from previous requests
        _last_web_search_results = []

        try:
            result = await Runner.run(
                starting_agent=self.agent,
                input=message,
                run_config=config,
                max_turns=15  # Increased from default 10
            )
            # With output_type=AgentResponse, final_output is already an AgentResponse
            if isinstance(result.final_output, AgentResponse):
                return self._inject_web_sources(result.final_output)
            # Fallback: wrap string response in AgentResponse
            return self._inject_web_sources(AgentResponse(response=str(result.final_output)))
        except MaxTurnsExceeded as e:
            # Gracefully handle turn limit - extract partial results from the run
            partial_response = self._extract_partial_response(e)
            if partial_response:
                response = AgentResponse(
                    response=f"📚 **Source: Documentation Knowledge Base**\n\n{partial_response}\n\n---\n*Note: Response was synthesized from available search results due to processing limits.*"
                )
                return self._inject_web_sources(response)
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
