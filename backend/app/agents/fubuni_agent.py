from agents import (
    Agent,
    Runner,
    AsyncOpenAI,
    OpenAIChatCompletionsModel,
    function_tool,
)
from agents.run import RunConfig
import os
from dotenv import load_dotenv
from openai.types.responses import ResponseTextDeltaEvent
from ..config.settings import settings
import asyncio

# Enable verbose logging for stdout
from agents import enable_verbose_stdout_logging

# debugging
enable_verbose_stdout_logging()  # Enable verbose logging for stdout
load_dotenv()

# Use OpenRouter API with Nova model
OPENROUTER_API_KEY = os.getenv("OPENROUTER_API_KEY")
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")

# Prioritize OpenRouter with Nova model
if OPENROUTER_API_KEY and not OPENROUTER_API_KEY.startswith("sk-or-v1-xxxxxxxx"):
    # Use OpenRouter API key
    api_key = OPENROUTER_API_KEY
    base_url = settings.openai_base_url
    model_name = "amazon/nova-2-lite-v1:free"
    use_mock = False
elif GEMINI_API_KEY:
    # Use Gemini API key as fallback
    api_key = GEMINI_API_KEY
    base_url = "https://generativelanguage.googleapis.com/v1beta/openai/"  # 📡 Gemini pretending to be OpenAi
    model_name = "gemini-2.0-flash"
    use_mock = False
else:
    # Mock mode - no valid API keys
    api_key = None
    base_url = None
    model_name = None
    use_mock = True

if not use_mock:
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
else:
    external_client = None
    model = None
    config = None


@function_tool
async def get_robotics_info(topic: str) -> str:
    """Fetch information about robotics topics. Useful for answering questions about humanoid robots, actuators, sensors, etc."""
    return f"Here's some information about {topic}: This is a robotics documentation platform. You can find detailed information about various robotics topics in the documentation."


class FubuniAgent:
    def __init__(self):
        self.agent = Agent(
            name="Fubuni",
            instructions="You are Fubuni, an AI assistant for the Physical Humanoid Robotics documentation platform. Always respond helpfully and accurately. If you don't know something, respond with 'I'm not sure yet, teach me!'. Be concise but informative in your responses. Use the get_robotics_info tool when users ask about specific robotics topics.",
            tools=[get_robotics_info],
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
    if use_mock:
        # Mock streaming response
        mock_response = f"Hello! I'm Fubuni, your AI assistant for Physical Humanoid Robotics. I see you're asking about: '{message}'. This is a mock response while we set up API keys. In a full implementation, I would provide detailed information about robotics topics including actuators, sensors, control systems, and path planning for humanoid robots."

        # Stream response word by word
        words = mock_response.split()
        for i, word in enumerate(words):
            yield word + (" " if i < len(words) - 1 else "")
            await asyncio.sleep(0.05)  # Small delay to simulate streaming
        return

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


# Global instance of the Fubuni agent
fubuni_agent = FubuniAgent()
