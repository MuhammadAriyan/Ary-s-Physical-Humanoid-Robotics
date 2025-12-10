---
id: 2
title: Create implementation plan
stage: plan
date: 2025-12-09
surface: agent
model: claude-sonnet-4-5-20250929
feature: fubuni-chat-bubble
branch: 007-fubuni-chat-bubble
user: ary
command: sp.plan
labels: ["plan", "fubuni", "architecture"]
links:
  spec: specs/007-fubuni-chat-bubble/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/007-fubuni-chat-bubble/plan.md
  - specs/007-fubuni-chat-bubble/research.md
  - specs/007-fubuni-chat-bubble/data-model.md
  - specs/007-fubuni-chat-bubble/contracts/chat-api.yaml
  - specs/007-fubuni-chat-bubble/quickstart.md
  - CLAUDE.md
tests:
  - none
---

## Prompt

```text
1. Python/FastAPI backend

   ├── /api/chat endpoint (POST + SSE streaming)

   ├── Agent created with OpenAI Agents SDK

   ├── Custom base_url + api_key (your provider)

   ├── System prompt: "You are Fubuni, a friendly and precise assistant..."

2. Docusaurus frontend

   ├── Swizzle ThemeClassic → inject FubuniChat component

   ├── Floating bubble → drawer → full-screen modal

   ├── All styling using only Infima classes (perfect theme match)

3. Build integration

   ├── Simple proxy or static JSON fallback for pure-static deploy

   ├── Or optional tiny function if needed   use  context 7 to read the openAI sdk and muhammadariyan/open-ai-sdk-journey and dont use openai agent use from agents import Agent,Runner,AsyncOpenAI,OpenAIChatCompletionsModel,function_tool,ModelSettings
from agents.run import RunConfig
from agents.agent import StopAtTools
from dotenv import load_dotenv
import os
from agents import enable_verbose_stdout_logging
from pydantic import BaseModel
enable_verbose_stdout_logging()  # Enable verbose logging for stdout
load_dotenv()

if not os.getenv("GEMINI_API_KEY"):
    raise ValueError("GEMINI_API_KEY environment variable is not set.")


@function_tool
async def punch(person:str):
    """
    Function to punch.
    """
    return f"ye le mukka kha! sun beta {person} mera mukaa khatarnak hai!"


@function_tool
async def kick(person:str):
    """
    Function to kick.
    """
    return f"ye le laat kha! sun beta {person} mera laat khatarnak hai!"



# Initialize the external client for Gemini
external_client = AsyncOpenAI(
    api_key=os.getenv("GEMINI_API_KEY"),
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/"  # 📡 Gemini pretending to be OpenAI
)

# Define the model using the external client
model = OpenAIChatCompletionsModel(
    openai_client=external_client,
    model="gemini-2.0-flash",
    # temperature=0.7,  # Uncomment for creativity
    # max_tokens=1000,  # Limit the response length
    # top_p=0.9,  # Use nucleus sampling
)

config = RunConfig(
    model=model,
    model_provider=external_client,)

# Creating a Output type
class CustomOutput(BaseModel):
    name:str
    quote:str
    joke:str


ModelSettingBase = ModelSettings(temperature=0.7,tool_choice='required',parallel_tool_calls=True)
ModelSettingFinal = ModelSettings(temperature=0.3)

# handsoff agent
champaklal = Agent(
    name="Champaklal",
    handoff_description="You are Champaklal from TMKOC (mimic him), a helpful dada and jhetia aka jhetalal ka baap. Respond to user queries with relevant information.",
    instructions="You are Champaklal from TMKOC, a helpful mota bnda and jhetia aka jhetalal ka baap and funny. Respond to user queries with relevant information.if someone says punch or mukka then use the punch tool and respond with the output of the tool. if someone says kick or laat then use the kick tool and respond with the output of the tool.",
    # output_type=CustomOutput,
    tools=[punch,kick],  # Add the punch function as a tool
    tool_use_behavior=StopAtTools(stop_at_tool_names=['punch'])  # tool pr a kar rok deta hai jo tool output dega bs wahi
)
tapu = champaklal.clone( #jo change krna hoga wo rewrite krdo
    name="Tapu",
    instructions=["You are Tapu from TMKOC (mimic him), a helpful mota bnda ka beta . Respond to user queries with relevant information. if talked about champaklal(aka chacha g or champak lal), then handsoff to champaklal agent",""],
    model_settings= ModelSettingBase.resolve(ModelSettingFinal)  # More creative responses,

)

jhetalal = Agent(
        name="Jhetalal",
        instructions="You are Jhetalal from TMKOC (mimic him), a helpful mota bnda . Respond to user queries with relevant information. if talked about champaklal(aka chacha g or champak lal) or if user seems to be talking to champak lal then handsoff to champaklal agent",
        output_type=CustomOutput,
        handoffs=[champaklal,tapu],  # Hand off to Champaklal agent
    )

async def main():

    result=await Runner.run(champaklal,'use punch tool to tapu and kick tool to jhetia', run_config=config)

    print(result.final_output)



if __name__ == "__main__":
    import asyncio
    asyncio.run(main())



# error but runs anyway :
# OPENAI_API_KEY is not set, skipping trace export

# eroor asli wale
# runtime warning runner was never awaited but for api key use openrouter api key i'll provide and use this url https://openrouter.ai/api/v1/chat/completions also donot forget to install the required things when ill implement these
```

## Response snapshot

Created comprehensive implementation plan for the Fubuni Chat Bubble feature including technical context, project structure, research findings, data models, API contracts, and quickstart guide. The plan details a FastAPI backend with SSE streaming and a Docusaurus frontend with swizzled components for seamless integration.

## Outcome

- ✅ Impact: Complete implementation plan with all required artifacts
- 🧪 Tests: none
- 📁 Files: plan.md, research.md, data-model.md, contracts/chat-api.yaml, quickstart.md, CLAUDE.md
- 🔁 Next prompts: Task generation and implementation
- 🧠 Reflection: Plan aligns with constitution and technical requirements

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Generate tasks from plan