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

# Always use OpenRouter (production ready)
if OPENROUTER_API_KEY:
    # Use OpenRouter API key
    api_key = OPENROUTER_API_KEY
    base_url = settings.openai_base_url
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


class FubuniAgent:
    def __init__(self):
        self.agent = Agent(
            name="Fubuni",
            instructions="You are Fubuni, an AI assistant for Physical Humanoid Robotics documentation platform. Always respond helpfully and accurately. If you don't know something, respond with 'I'm not sure yet, teach me!'. Be concise but informative in your responses. Use the get_robotics_info tool when users ask about specific robotics topics.",
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


# Global instance of Fubuni agent
fubuni_agent = FubuniAgent()
