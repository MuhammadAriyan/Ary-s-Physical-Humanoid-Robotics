---
title: "Conversational Robotics"
sidebar_position: 6
---

# Chapter 6: Conversational Robotics

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the principles of conversational AI and its application to humanoid robots
- Integrate large language models like GPT for natural dialogue generation
- Implement speech recognition and text-to-speech pipelines for voice interaction
- Design natural language understanding systems that parse user intent
- Create multi-modal interaction systems combining speech, gesture, and vision
- Build complete dialogue systems that enable meaningful human-robot conversation

## 6.1 Introduction to Conversational AI in Robotics

Conversational AI represents a fundamental shift in how humans interact with robots. Rather than relying on joysticks, buttons, or complex programming interfaces, conversational AI enables natural spoken dialogue as the primary interaction modality. For humanoid robots designed to operate in human environments, this capability is essential for seamless integration into domestic, healthcare, and service settings.

The evolution from command-based interfaces to conversational interaction reflects broader trends in human-computer interaction. Early robotic systems required users to learn specific command syntax or button combinations. Modern conversational interfaces accept natural language, allowing users to communicate intentions without specialized training. This democratization of robot control opens humanoid robotics to elderly users, children, and individuals without technical backgrounds.

Humanoid robots present unique opportunities for conversational AI due to their anthropomorphic form. When a robot can speak naturally, gesture expressively, and maintain eye contact during conversation, the interaction feels more intuitive and engaging. A humanoid robot asking "Would you like me to bring you a cup of tea?" is far more natural than a screen-based assistant presenting the same question. The embodied nature of the interaction creates expectations of social intelligence that conversational AI must fulfill.

The technical challenges of conversational robotics extend beyond simple speech processing. A conversational humanoid robot must maintain context across extended interactions, understand ambiguous references to objects and people in the environment, generate appropriate responses that match the robot's personality and capabilities, and synchronize verbal output with non-verbal cues like facial expressions and gestures. These requirements demand integration across multiple AI subsystems including speech recognition, natural language understanding, dialogue management, natural language generation, and speech synthesis.

### The Conversational AI Pipeline

Conversational AI for robots involves a sophisticated pipeline of processing stages, each building upon the outputs of the previous stage. Understanding this pipeline is essential for designing robust dialogue systems that can handle the variability of real-world conversation.

The pipeline begins with **automatic speech recognition** (ASR), which converts raw audio input into text transcripts. Modern ASR systems use deep neural networks trained on massive speech corpora to achieve high accuracy across diverse speakers and acoustic conditions. For robot applications, ASR must handle background noise, multiple speakers, and the acoustic properties of the robot's physical environment.

Following speech recognition, **natural language understanding** (NLU) parses the transcribed text to extract semantic meaning. NLU identifies the user's intent, extracts relevant entities, and resolves references to objects or people in context. For a command like "Bring me the red cup from the kitchen," NLU identifies the intent as a request for object delivery, extracts the object type (cup), color (red), and location (kitchen).

The **dialogue manager** maintains conversation state and determines appropriate system responses. It tracks what has been said, what the current topic is, and what information has been established. The dialogue manager also handles dialogue acts like asking for clarification, confirming understanding, or providing information.

**Natural language generation** (NLG) creates the textual response that the system will speak. Modern approaches use large language models for NLG, producing fluent text that matches the conversation context and maintains a consistent persona. NLG must also consider the robot's personality and the appropriateness of different response styles.

Finally, **text-to-speech** (TTS) synthesis converts the generated text into audible speech. Modern TTS systems using neural network architectures produce remarkably natural-sounding speech with appropriate prosody and emotional tone. The TTS output must also be synchronized with the robot's lip movements and facial expressions for natural embodiment.

### See Also

- **Part 1**: [Introduction to Physical AI](part-1-foundations/introduction-to-physical-ai) covers foundational concepts
- **Part 3**: [Gazebo and Unity Simulation](part-3-simulation/gazebo-unity-simulation) discusses sensor simulation for perception
- **Part 5**: [Humanoid Development](part-5-humanoid/humanoid-robot-development) covers humanoid robot design

## 6.2 Integrating GPT Models for Natural Dialogue

Large language models have revolutionized conversational AI by providing unprecedented capabilities for natural language understanding and generation. Models in the GPT family, developed by OpenAI and others, can engage in coherent, contextually appropriate dialogue across a wide range of topics. Integrating these models into robot systems enables natural conversation while maintaining appropriate safety constraints and task focus.

The integration of GPT models into robot dialogue systems requires careful architecture design. Direct connection to cloud-based language models introduces latency that can make conversation feel unnatural. Network dependencies also raise reliability concerns for safety-critical applications. Successful integration strategies typically involve a hybrid approach where lightweight local models handle routine interactions while cloud models provide specialized knowledge when needed.

The following example demonstrates a dialogue manager that interfaces with a GPT model for response generation while maintaining robot-specific context and safety constraints:

```python
#!/usr/bin/env python3
"""
GPT-Powered Dialogue Manager for Humanoid Robots

This module provides a complete dialogue management system that integrates
large language models with robot-specific context and constraints.
"""

import asyncio
import json
import os
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum, auto
from typing import Any, Dict, List, Optional, Tuple
from abc import ABC, abstractmethod


class DialogueState(Enum):
    """Enumeration of possible dialogue states."""
    IDLE = auto()
    LISTENING = auto()
    PROCESSING = auto()
    SPEAKING = auto()
    WAITING_FOR_ACTION = auto()
    ERROR = auto()


class Intent(Enum):
    """Robot-specific dialogue intents."""
    GREETING = "greeting"
    FAREWELL = "farewell"
    QUESTION = "question"
    COMMAND = "command"
    REQUEST = "request"
    INFORMATION = "information"
    CONFIRMATION = "confirmation"
    APOLOGY = "apology"
    UNKNOWN = "unknown"


@dataclass
class Entity:
    """Represents an extracted entity from user input."""
    type: str
    value: str
    confidence: float
    position: Tuple[int, int]  # Start and end character positions


@dataclass
class DialogueAct:
    """Represents a complete dialogue act with intent and entities."""
    intent: Intent
    entities: List[Entity] = field(default_factory=list)
    confidence: float = 1.0
    raw_text: str = ""
    timestamp: datetime = field(default_factory=datetime.now)


@dataclass
class ConversationContext:
    """Maintains context across a conversation session."""
    session_id: str
    user_id: Optional[str]
    robot_name: str = "Fubuni"
    conversation_history: List[Dict[str, Any]] = field(default_factory=list)
    known_entities: Dict[str, Any] = field(default_factory=dict)
    current_topic: Optional[str] = None
    last_action_result: Optional[str] = None
    user_preferences: Dict[str, Any] = field(default_factory=dict)

    def add_exchange(self, user_message: str, robot_response: str) -> None:
        """Add a user-robot exchange to conversation history."""
        self.conversation_history.append({
            "timestamp": datetime.now().isoformat(),
            "user": user_message,
            "robot": robot_response
        })
        # Keep history manageable for context windows
        max_history = 20
        if len(self.conversation_history) > max_history:
            self.conversation_history = self.conversation_history[-max_history:]


class GPTClient:
    """
    Client for interfacing with GPT-based language models.

    Handles prompt construction, API communication, and response parsing
    while maintaining safety constraints and context management.
    """

    def __init__(
        self,
        api_key: Optional[str] = None,
        model: str = "gpt-4",
        max_tokens: int = 500,
        temperature: float = 0.7,
        system_prompt: Optional[str] = None
    ):
        self.api_key = api_key or os.getenv("OPENAI_API_KEY")
        self.model = model
        self.max_tokens = max_tokens
        self.temperature = temperature
        self.system_prompt = system_prompt or self._get_default_system_prompt()
        self._client = None

    def _get_default_system_prompt(self) -> str:
        """Generate the default system prompt for robot dialogue."""
        return f"""You are Fubuni, a friendly humanoid robot assistant. You are helpful,
polite, and concise in your responses. You assist users with tasks in a home
or office environment. You can:
- Answer questions about your capabilities
- Help with object location and retrieval
- Provide information about the environment
- Execute simple commands like navigation and manipulation
- Engage in casual conversation

Keep responses brief (1-3 sentences) unless more detail is requested.
Always maintain a helpful, positive tone. If you cannot help with a request,
explain why clearly and suggest alternatives."""

    def _build_messages(
        self,
        context: ConversationContext,
        user_input: str
    ) -> List[Dict[str, str]]:
        """Build the message list for the API request."""
        messages = [{"role": "system", "content": self.system_prompt}]

        # Add conversation history for context
        for exchange in context.conversation_history[-5:]:  # Last 5 exchanges
            messages.append({"role": "user", "content": exchange["user"]})
            messages.append({"role": "assistant", "content": exchange["robot"]})

        # Add current user input
        messages.append({"role": "user", "content": user_input})

        return messages

    async def generate_response(
        self,
        context: ConversationContext,
        user_input: str
    ) -> str:
        """
        Generate a response using the GPT model.

        Args:
            context: Current conversation context
            user_input: The user's input text

        Returns:
            Generated response text
        """
        messages = self._build_messages(context, user_input)

        # Simulated response for demonstration
        # In production, this would call the actual API
        response = await self._call_api(messages)

        return response

    async def _call_api(self, messages: List[Dict[str, str]]) -> str:
        """Make the API call to the language model."""
        # Placeholder for actual API integration
        # Would use openai.AsyncOpenAI client in production
        await asyncio.sleep(0.1)  # Simulate API latency

        user_message = messages[-1]["content"] if messages else ""

        # Simple response generation for demonstration
        response_templates = {
            "greeting": ["Hello! How can I help you today?", "Hi there! What can I do for you?"],
            "command": ["I understand. Let me help with that.", "Got it! I'll take care of that."],
            "question": ["That's a great question. Let me think about that.", "I'd be happy to answer that."],
            "farewell": ["Goodbye! Feel free to ask if you need anything else.", "See you later!"]
        }

        # Simple intent-based response (production would use actual NLU)
        if any(g in user_message.lower() for g in ["hello", "hi", "hey"]):
            return "Hello! I'm Fubuni, your humanoid robot assistant. How can I help you today?"
        elif any(q in user_message.lower() for q in ["what", "how", "why", "where", "when"]):
            return "That's a great question. I'm here to help you with tasks around the home or office. What would you like to know?"
        elif any(c in user_message.lower() for c in ["bring", "get", "find", "fetch"]):
            return "I understand you need help with something. Could you tell me what object you're looking for?"
        elif any(f in user_message.lower() for f in ["bye", "goodbye", "see you"]):
            return "Goodbye! It was nice chatting with you. Feel free to return if you need anything!"
        else:
            return "I'm here to help! What would you like me to do for you?"


class NaturalLanguageUnderstanding:
    """
    NLU module for intent classification and entity extraction.

    Extracts user intent and relevant entities from transcribed speech,
    enabling appropriate dialogue management and robot action selection.
    """

    def __init__(self):
        self.intent_classifier = IntentClassifier()
        self.entity_extractor = EntityExtractor()

    def parse(self, text: str) -> DialogueAct:
        """
        Parse user input into a structured dialogue act.

        Args:
            text: The transcribed user input

        Returns:
            DialogueAct with intent and entities
        """
        intent = self.intent_classifier.classify(text)
        entities = self.entity_extractor.extract(text)

        return DialogueAct(
            intent=intent,
            entities=entities,
            raw_text=text,
            confidence=self._calculate_confidence(intent, entities)
        )

    def _calculate_confidence(
        self,
        intent: Intent,
        entities: List[Entity]
    ) -> float:
        """Calculate overall confidence score for the parse."""
        base_confidence = 0.9 if intent != Intent.UNKNOWN else 0.5

        # Boost confidence if entities were found for relevant intents
        entity_boost = min(0.1 * len(entities), 0.1)

        return min(base_confidence + entity_boost, 1.0)


class IntentClassifier:
    """
    Simple intent classifier based on keyword matching.

    Production systems would use trained classifiers or LLM-based
    classification for more robust intent recognition.
    """

    INTENT_PATTERNS = {
        Intent.GREETING: ["hello", "hi", "hey", "good morning", "good afternoon"],
        Intent.FAREWELL: ["bye", "goodbye", "see you", "later", "farewell"],
        Intent.COMMAND: ["move", "go", "walk", "stop", "turn", "grab", "pick"],
        Intent.REQUEST: ["bring", "get", "find", "fetch", "bring me", "please"],
        Intent.QUESTION: ["what", "how", "why", "where", "when", "who", "which"],
        Intent.INFORMATION: ["tell me", "explain", "describe", "information"],
        Intent.CONFIRMATION: ["yes", "confirm", "correct", "that's right"],
        Intent.APOLOGY: ["sorry", "apologize", "excuse me", "pardon"]
    }

    def classify(self, text: str) -> Intent:
        """Classify the intent of user input."""
        text_lower = text.lower()

        for intent, patterns in self.INTENT_PATTERNS.items():
            if any(pattern in text_lower for pattern in patterns):
                return intent

        return Intent.UNKNOWN


class EntityExtractor:
    """
    Entity extractor for common entity types in robot interactions.

    Identifies objects, locations, people, and quantities from natural
    language input for action execution.
    """

    ENTITY_TYPES = {
        "object": ["cup", "bottle", "book", "phone", "keys", "bag", "box", "pen"],
        "location": ["kitchen", "living room", "bedroom", "office", "bathroom", "table", "desk"],
        "person": ["me", "you", "him", "her", "them", "my wife", "my husband"],
        "quantity": ["one", "two", "three", "first", "second", "a", "the"]
    }

    def extract(self, text: str) -> List[Entity]:
        """Extract entities from user input."""
        entities = []
        text_lower = text.lower()

        for entity_type, keywords in self.ENTITY_TYPES.items():
            for keyword in keywords:
                if keyword in text_lower:
                    position = text_lower.find(keyword)
                    entities.append(Entity(
                        type=entity_type,
                        value=keyword,
                        confidence=0.85,
                        position=(position, position + len(keyword))
                    ))

        return entities


class DialogueManager:
    """
    Main dialogue manager coordinating all conversational AI components.

    Manages the dialogue state machine, integrates with the GPT model,
    and coordinates with robot action systems for task execution.
    """

    def __init__(
        self,
        gpt_client: GPTClient,
        nlu: NaturalLanguageUnderstanding,
        robot_name: str = "Fubuni"
    ):
        self.gpt_client = gpt_client
        self.nlu = nlu
        self.robot_name = robot_name
        self.state = DialogueState.IDLE
        self.context: Optional[ConversationContext] = None

    async def start_conversation(
        self,
        session_id: str,
        user_id: Optional[str] = None
    ) -> str:
        """Start a new conversation session."""
        self.context = ConversationContext(
            session_id=session_id,
            user_id=user_id,
            robot_name=self.robot_name
        )
        self.state = DialogueState.LISTENING

        return f"Hello! I'm {self.robot_name}, your humanoid robot assistant. How can I help you today?"

    async def process_input(self, user_input: str) -> str:
        """
        Process user input and generate an appropriate response.

        This is the main entry point for the dialogue system, handling
        the complete pipeline from input to response.
        """
        if self.context is None:
            raise RuntimeError("Conversation not started. Call start_conversation first.")

        self.state = DialogueState.PROCESSING

        try:
            # Step 1: Natural Language Understanding
            dialogue_act = self.nlu.parse(user_input)

            # Step 2: Generate response using GPT
            response = await self.gpt_client.generate_response(
                self.context,
                user_input
            )

            # Step 3: Update conversation context
            self.context.add_exchange(user_input, response)
            self._update_context_from_act(dialogue_act)

            self.state = DialogueState.SPEAKING

            return response

        except Exception as e:
            self.state = DialogueState.ERROR
            return f"I'm sorry, I didn't understand that. Could you please try again?"

    def _update_context_from_act(self, act: DialogueAct) -> None:
        """Update conversation context based on parsed dialogue act."""
        # Update current topic based on intent
        if act.intent == Intent.COMMAND or act.intent == Intent.REQUEST:
            # Extract object entities for task context
            for entity in act.entities:
                if entity.type == "object":
                    self.context.known_entities["target_object"] = entity.value
                elif entity.type == "location":
                    self.context.known_entities["target_location"] = entity.value


# Example usage and demonstration
async def main():
    """Demonstrate the dialogue system with example interactions."""

    # Initialize components
    gpt_client = GPTClient()
    nlu = NaturalLanguageUnderstanding()
    dialogue_manager = DialogueManager(gpt_client, nlu)

    # Start conversation
    print("=== Fubuni Dialogue System Demo ===\n")

    welcome = await dialogue_manager.start_conversation(
        session_id="demo-session-001",
        user_id="guest"
    )
    print(f"Robot: {welcome}\n")

    # Example user inputs
    user_inputs = [
        "Hello Fubuni, can you bring me a cup of water?",
        "It's on the kitchen table.",
        "Thank you so much!",
        "Goodbye for now!"
    ]

    for user_input in user_inputs:
        print(f"User: {user_input}")
        response = await dialogue_manager.process_input(user_input)
        print(f"Robot: {response}\n")

        # Check current state
        print(f"[Dialogue State: {dialogue_manager.state.name}]")
        print(f"[Context: {len(dialogue_manager.context.conversation_history)} exchanges]\n")


if __name__ == "__main__":
    asyncio.run(main())
```

This example demonstrates the key architecture patterns for GPT integration in robot dialogue systems. The system maintains conversation context, parses user intent, generates appropriate responses, and can be extended to execute physical actions based on user commands.

## 6.3 Speech Recognition and Text-to-Speech

Speech recognition and speech synthesis form the voice interface layer of conversational robotics. These technologies enable robots to receive verbal commands and provide verbal feedback, creating a natural two-way communication channel. For humanoid robots operating in human environments, voice interaction provides accessibility advantages and supports the social expectations that come with humanoid embodiment.

### Automatic Speech Recognition

Modern automatic speech recognition systems use deep neural networks to convert audio waveforms into text transcripts. The key architectural components include acoustic models that map audio features to phonemes, language models that provide probabilistic context for word sequences, and pronunciation models that connect words to their phonetic representations. End-to-end models like Whisper from OpenAI have simplified this architecture by training single neural networks that directly map audio to text.

For robot applications, speech recognition must handle several practical challenges. Background noise from ventilation, other people, or robot motors can degrade recognition accuracy. Far-field microphone configurations designed for room-scale audio capture introduce reverberation that distorts the acoustic signal. Multiple simultaneous speakers require source separation or speaker diarization to attribute speech correctly.

The following example implements a speech recognition node for ROS 2 that integrates with common ASR services:

```python
#!/usr/bin/env python3
"""
Speech Recognition Module for Humanoid Robots

Provides real-time speech recognition integration with ROS 2,
supporting both local and cloud-based ASR services.
"""

import asyncio
import audioop
import os
import threading
import wave
from abc import ABC, abstractmethod
from collections import deque
from dataclasses import dataclass
from typing import Any, Callable, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import AudioData
from rclpy.qos import QoSProfile, ReliabilityPolicy


@dataclass
class SpeechRecognitionResult:
    """Result from speech recognition processing."""
    transcript: str
    confidence: float
    is_final: bool
    timestamp: float
    audio_energy: float


class AudioPreprocessor:
    """
    Audio preprocessing pipeline for speech recognition.

    Handles audio normalization, noise reduction, and feature extraction
    to improve recognition accuracy in challenging acoustic environments.
    """

    def __init__(
        self,
        sample_rate: int = 16000,
        chunk_size: int = 1024,
        noise_reduction: bool = True,
        normalize_audio: bool = True
    ):
        self.sample_rate = sample_rate
        self.chunk_size = chunk_size
        self.noise_reduction = noise_reduction
        self.normalize_audio = normalize_audio
        self.audio_buffer = deque(maxlen=10)  # Keep last 10 chunks for VAD
        self.energy_threshold = 500.0  # Voice activity detection threshold
        self.adaptive_threshold = True

    def calculate_energy(self, audio_data: bytes) -> float:
        """Calculate root mean square energy of audio chunk."""
        try:
            rms = audioop.rms(audio_data, 2)  # 2 bytes per sample for 16-bit audio
            return float(rms)
        except Exception:
            return 0.0

    def normalize(self, audio_data: bytes) -> bytes:
        """Normalize audio to consistent volume level."""
        try:
            max_amplitude = max(audioop.max(audio_data, 2), 1)
            scale_factor = 32767.0 / max_amplitude
            normalized = audioop.mul(audio_data, 2, scale_factor)
            return normalized
        except Exception:
            return audio_data

    def apply_noise_reduction(self, audio_data: bytes) -> bytes:
        """
        Simple noise reduction using spectral subtraction.

        In production, more sophisticated algorithms like Wiener filtering
        or deep learning-based noise suppression would be used.
        """
        # Placeholder for noise reduction
        # Production implementations would use libraries like noisereduce
        return audio_data

    def preprocess(self, audio_data: bytes) -> Tuple[bytes, float]:
        """
        Preprocess audio chunk for recognition.

        Returns:
            Tuple of (processed_audio, energy_level)
        """
        energy = self.calculate_energy(audio_data)

        # Adaptive threshold adjustment
        if self.adaptive_threshold:
            self.energy_threshold = 0.9 * self.energy_threshold + 0.1 * energy

        # Apply preprocessing steps
        if self.normalize_audio:
            audio_data = self.normalize(audio_data)

        if self.noise_reduction:
            audio_data = self.apply_noise_reduction(audio_data)

        return audio_data, energy

    def is_speech(self, audio_data: bytes) -> bool:
        """Voice activity detection using energy threshold."""
        energy = self.calculate_energy(audio_data)
        return energy > self.energy_threshold


class ASRProvider(ABC):
    """Abstract base class for ASR service providers."""

    @abstractmethod
    async def recognize(self, audio_data: bytes) -> SpeechRecognitionResult:
        """Perform speech recognition on audio data."""
        pass


class WhisperASR(ASRProvider):
    """
    Whisper-based speech recognition using OpenAI's Whisper model.

    Supports both local (Whisper.cpp) and cloud (OpenAI API) deployments.
    """

    def __init__(
        self,
        model: str = "base",
        language: str = "en",
        use_cloud: bool = False,
        api_key: Optional[str] = None
    ):
        self.model = model
        self.language = language
        self.use_cloud = use_cloud
        self.api_key = api_key or os.getenv("OPENAI_API_KEY")

    async def recognize(self, audio_data: bytes) -> SpeechRecognitionResult:
        """
        Perform speech recognition using Whisper.

        This is a simplified implementation. Production code would:
        1. Save audio to temporary file (WAV format)
        2. Call whisper.cpp via subprocess or OpenAI API
        3. Parse and return results
        """
        import time
        timestamp = time.time()

        # Placeholder implementation
        # In production, this would interface with whisper.cpp or OpenAI API
        await asyncio.sleep(0.05)  # Simulate processing time

        return SpeechRecognitionResult(
            transcript="",  # Would contain actual transcription
            confidence=0.0,
            is_final=False,
            timestamp=timestamp,
            audio_energy=0.0
        )


class ROS2SpeechRecognizer(Node):
    """
    ROS 2 node for speech recognition integration.

    Subscribes to audio topics, processes audio through the ASR pipeline,
    and publishes recognition results.
    """

    def __init__(
        self,
        asr_provider: ASRProvider,
        audio_preprocessor: AudioPreprocessor,
        node_name: str = "speech_recognizer"
    ):
        super().__init__(node_name)

        self.asr_provider = asr_provider
        self.preprocessor = audio_preprocessor

        # Configuration
        self.declare_parameter("confidence_threshold", 0.6)
        self.declare_parameter("partial_results_enabled", True)
        self.declare_parameter("silence_duration", 0.8)  # Seconds before finalizing

        self.confidence_threshold = self.get_parameter("confidence_threshold").value
        self.partial_enabled = self.get_parameter("partial_results_enabled").value

        # State
        self.audio_buffer: List[bytes] = []
        self.is_speaking = False
        self.silence_timer: Optional[float] = None

        # QoS profile for audio topics
        self.audio_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Publishers
        self.transcript_pub = self.create_publisher(
            String,
            "speech/transcript",
            10
        )

        self.confidence_pub = self.create_publisher(
            Float32,
            "speech/confidence",
            10
        )

        self.status_pub = self.create_publisher(
            String,
            "speech/status",
            10
        )

        # Subscriber
        self.audio_sub = self.create_subscription(
            AudioData,
            "audio/raw",
            self.audio_callback,
            self.audio_qos
        )

        self.get_logger().info("Speech Recognizer initialized")

    def audio_callback(self, msg: AudioData) -> None:
        """Process incoming audio data."""
        audio_data = bytes(msg.data)
        processed_audio, energy = self.preprocessor.preprocess(audio_data)

        # Voice activity detection
        is_speech = self.preprocessor.is_speech(processed_audio)

        if is_speech:
            self.audio_buffer.append(processed_audio)
            self.is_speaking = True
            self.silence_timer = None
        else:
            if self.is_speaking and self.audio_buffer:
                # Silence detected after speech - process utterance
                if self.silence_timer is None:
                    self.silence_timer = self.get_clock().now().nanoseconds / 1e9
                else:
                    silence_duration = (
                        self.get_clock().now().nanoseconds / 1e9 - self.silence_timer
                    )
                    if silence_duration > self.get_parameter("silence_duration").value:
                        self._process_utterance()

    def _process_utterance(self) -> None:
        """Process accumulated audio buffer as complete utterance."""
        if not self.audio_buffer:
            return

        # Combine audio chunks
        audio_data = b"".join(self.audio_buffer)
        self.audio_buffer = []
        self.is_speaking = False

        # Perform recognition
        result = asyncio.run_coroutine_threadsafe(
            self.asr_provider.recognize(audio_data),
            asyncio.new_event_loop()
        ).result()

        # Publish results
        if result.confidence >= self.confidence_threshold:
            transcript_msg = String()
            transcript_msg.data = result.transcript
            self.transcript_pub.publish(transcript_msg)

            confidence_msg = Float32()
            confidence_msg.data = float(result.confidence)
            self.confidence_pub.publish(confidence_msg)

            self.get_logger().info(
                f"Recognized: '{result.transcript}' (confidence: {result.confidence:.2f})"
            )

        status_msg = String()
        status_msg.data = "listening"
        self.status_pub.publish(status_msg)


class TextToSpeechEngine:
    """
    Text-to-speech synthesis engine for robot speech output.

    Supports multiple TTS providers with voice customization options
    for different interaction contexts.
    """

    def __init__(self, default_voice: str = "en-US-Female"):
        self.default_voice = default_voice
        self.voice_cache: Dict[str, Any] = {}

    async def synthesize(
        self,
        text: str,
        voice: Optional[str] = None,
        speed: float = 1.0,
        pitch: float = 1.0
    ) -> bytes:
        """
        Synthesize speech from text.

        Args:
            text: Text to synthesize
            voice: Voice identifier (language, speaker, etc.)
            speed: Speech rate multiplier (0.5 to 2.0)
            pitch: Pitch multiplier (0.5 to 2.0)

        Returns:
            Audio data in WAV format
        """
        # Placeholder implementation
        # Production would use gTTS, pyttsx3, or cloud TTS services
        await asyncio.sleep(0.1)

        # Return empty bytes placeholder
        return b""

    def preprocess_text(self, text: str) -> str:
        """Preprocess text for better speech synthesis."""
        # Expand abbreviations
        replacements = {
            "Dr.": "Doctor",
            "Mr.": "Mister",
            "Mrs.": "Missus",
            "etc.": "et cetera",
            "i.e.": "that is",
            "e.g.": "for example"
        }

        processed = text
        for abbr, full in replacements.items():
            processed = processed.replace(abbr, full)

        # Add pauses for punctuation
        processed = processed.replace(".", ". ")
        processed = processed.replace("?", "? ")
        processed = processed.replace("!", "! ")

        return processed.strip()


class ROS2SpeechSynthesizer(Node):
    """
    ROS 2 node for text-to-speech synthesis.

    Subscribes to text topics and publishes synthesized audio.
    """

    def __init__(self, tts_engine: TextToSpeechEngine):
        super().__init__("speech_synthesizer")

        self.tts_engine = tts_engine

        # QoS profile
        self.text_qos = QoSProfile(depth=10)

        # Publisher for audio
        self.audio_pub = self.create_publisher(
            AudioData,
            "speech/audio",
            10
        )

        # Subscriber for text input
        self.text_sub = self.create_subscription(
            String,
            "speech/text",
            self.synthesize_callback,
            self.text_qos
        )

        self.get_logger().info("Speech Synthesizer initialized")

    def synthesize_callback(self, msg: String) -> None:
        """Process text and synthesize speech."""
        text = msg.data

        # Preprocess text
        text = self.tts_engine.preprocess_text(text)

        # Synthesize
        audio_data = asyncio.run(
            self.tts_engine.synthesize(text)
        )

        if audio_data:
            # Publish audio
            audio_msg = AudioData()
            audio_msg.data = audio_data
            self.audio_pub.publish(audio_msg)

            self.get_logger().info(f"Synthesized speech for: '{text[:50]}...'")


# Demonstration of the speech recognition pipeline
async def demonstrate_speech_pipeline():
    """Demonstrate the speech recognition and synthesis pipeline."""

    print("=== Speech Recognition Pipeline Demo ===\n")

    # Initialize components
    preprocessor = AudioPreprocessor(
        sample_rate=16000,
        chunk_size=1024,
        noise_reduction=True,
        normalize_audio=True
    )

    asr_provider = WhisperASR(model="base", language="en")

    print("Audio Preprocessor initialized:")
    print(f"  - Sample rate: {preprocessor.sample_rate} Hz")
    print(f"  - Chunk size: {preprocessor.chunk_size} samples")
    print(f"  - Noise reduction: {preprocessor.noise_reduction}")
    print(f"  - Adaptive VAD threshold: {preprocessor.adaptive_threshold}\n")

    print("Whisper ASR initialized:")
    print(f"  - Model: {asr_provider.model}")
    print(f"  - Language: {asr_provider.language}")
    print(f"  - Cloud mode: {asr_provider.use_cloud}\n")

    # TTS demonstration
    tts = TextToSpeechEngine(default_voice="en-US-Female")
    print("Text-to-Speech engine initialized:")
    print(f"  - Default voice: {tts.default_voice}\n")

    print("Pipeline components ready for ROS 2 integration.")
    print("Would subscribe to audio topics and publish recognized text.")


if __name__ == "__main__":
    asyncio.run(demonstrate_speech_pipeline())
```

## 6.4 Natural Language Understanding

Natural language understanding transforms raw transcribed text into structured representations that robot systems can act upon. The core tasks include intent classification, entity extraction, reference resolution, and dialogue act interpretation. For robot applications, NLU must be robust to the imprecise and incomplete language that characterizes natural human communication.

Intent classification assigns user utterances to predefined categories that map to robot actions or dialogue behaviors. Common intents for humanoid robots include greetings, commands, questions, requests for information, and conversation management acts like asking for clarification or confirming understanding. Multi-label classification allows utterances to carry multiple intents simultaneously, such as a greeting combined with a request.

Entity extraction identifies specific pieces of information within user utterances. For robot applications, the most important entity types are objects the robot might manipulate, locations in the environment, people who interact with the robot, and temporal expressions that specify when actions should occur. Entity extraction must handle both explicit mentions and references to previously discussed entities.

Reference resolution connects pronouns, demonstratives, and other anaphoric expressions to their referent entities. When a user says "Bring me the cup and put it on the table," the pronoun "it" refers to the cup, not the table. Reference resolution requires tracking discourse entities across conversation turns and understanding the physical context in which references occur.

The following example implements a comprehensive NLU pipeline with intent classification, entity extraction, and reference resolution:

```python
#!/usr/bin/env python3
"""
Natural Language Understanding for Humanoid Robots

Provides comprehensive NLU capabilities including intent classification,
entity extraction, reference resolution, and semantic parsing.
"""

import re
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from typing import Any, Dict, List, Optional, Set, Tuple


class Intent(Enum):
    """Comprehensive set of robot interaction intents."""
    # Core interaction intents
    GREETING = "greeting"
    FAREWELL = "farewell"
    THANK = "thank"
    APOLOGIZE = "apologize"

    # Task-oriented intents
    COMMAND = "command"  # Direct robot command
    REQUEST = "request"  # Polite request
    QUESTION = "question"
    SUGGESTION = "suggestion"

    # Information intents
    INFORM = "inform"
    CONFIRM = "confirm"
    DENY = "deny"

    # Dialogue management
    CLARIFY = "clarify"  # Ask for clarification
    REPEAT = "repeat"
    CHANGE_TOPIC = "change_topic"

    # Social
    COMPLIMENT = "compliment"
    COMPLAINT = "complaint"

    # Error/unknown
    UNKNOWN = "unknown"
    NON_SPEECH = "non_speech"


@dataclass
class Entity:
    """Extracted entity with type, value, and metadata."""
    type: str
    value: str
    raw_text: str
    position: Tuple[int, int]
    confidence: float = 1.0
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class Reference:
    """Anaphoric reference to a previously mentioned entity."""
    text: str
    position: Tuple[int, int]
    referent_type: str
    antecedent: Optional[str] = None
    confidence: float = 1.0


@dataclass
class NLUParsing:
    """Complete NLU parsing result."""
    text: str
    intent: Intent
    confidence: float
    entities: List[Entity]
    references: List[Reference]
    sentiment: float  # -1.0 to 1.0
    urgency: float  # 0.0 to 1.0
    politeness: float  # 0.0 to 1.0
    timestamp: datetime = field(default_factory=datetime.now)


class EntityExtractor:
    """
    Multi-type entity extractor for robot interaction contexts.

    Extracts objects, locations, people, quantities, times, and actions
    from natural language input using pattern matching and classification.
    """

    # Entity patterns
    ENTITY_PATTERNS = {
        "object": [
            r"\b(cup|mug|glass|bottle|plate|bowl)\b",
            r"\b(book|phone|laptop|tablet|remote)\b",
            r"\b(keys|wallet|sunglasses|hat)\b",
            r"\b(food|drink|snack|meal)\b",
            r"\b(newspaper|magazine|letter)\b"
        ],
        "location": [
            r"\b(kitchen|living room|bedroom|bathroom|office)\b",
            r"\b(table|desk|shelf|counter|cabinet)\b",
            r"\b(floor|ceiling|wall|door|window)\b",
            r"\bnear\s+\w+|next to\s+\w+|on top of\s+\w+"
        ],
        "person": [
            r"\b(me|you|him|her|them)\b",
            r"\b(my\s+\w+|your\s+\w+|his\s+\w+|her\s+\w+)\b",
            r"\b(mom|dad|friend|colleague)\b"
        ],
        "quantity": [
            r"\b(one|two|three|four|five)\b",
            r"\b(a|an|the)\b",
            r"\b(some|few|several|many)\b"
        ],
        "time": [
            r"\b(now|immediately|right away)\b",
            r"\b(later|today|tonight|tomorrow)\b",
            r"\b(in\s+\w+\s+(minutes?|hours?|seconds?))\b"
        ],
        "action": [
            r"\b(bring|get|find|fetch|grab)\b",
            r"\b(pick\s+up|put\s+down|set\s+down)\b",
            r"\b(move|go|walk|come|follow)\b",
            r"\b(open|close|turn|switch)\b"
        ]
    }

    def __init__(self):
        self.entity_values = self._build_entity_values()
        self.mention_history: List[Dict[str, Any]] = []

    def _build_entity_values(self) -> Dict[str, Set[str]]:
        """Build sets of known entity values for fuzzy matching."""
        return {
            "object": {
                "cup", "mug", "glass", "bottle", "plate", "bowl",
                "book", "phone", "laptop", "tablet", "remote",
                "keys", "wallet", "pen", "notebook", "bag"
            },
            "location": {
                "kitchen", "living room", "bedroom", "bathroom", "office",
                "table", "desk", "shelf", "counter", "cabinet"
            },
            "person": {
                "me", "you", "him", "her", "them"
            }
        }

    def extract(self, text: str) -> List[Entity]:
        """Extract all entities from text."""
        entities = []
        text_lower = text.lower()

        for entity_type, patterns in self.ENTITY_PATTERNS.items():
            for pattern in patterns:
                for match in re.finditer(pattern, text_lower):
                    value = match.group().strip()

                    # Clean up quantified expressions
                    if entity_type == "quantity" and value in ["a", "an"]:
                        continue

                    entity = Entity(
                        type=entity_type,
                        value=self._normalize_entity_value(entity_type, value),
                        raw_text=match.group(),
                        position=(match.start(), match.end()),
                        confidence=self._calculate_confidence(entity_type, value)
                    )
                    entities.append(entity)

        # Sort by position
        entities.sort(key=lambda e: e.position[0])

        return entities

    def _normalize_entity_value(self, entity_type: str, value: str) -> str:
        """Normalize extracted entity values."""
        # Remove articles and quantifiers
        stopwords = {"a", "an", "the", "some", "few", "several", "many"}
        words = value.split()
        filtered = [w for w in words if w not in stopwords]
        return " ".join(filtered) if filtered else value

    def _calculate_confidence(self, entity_type: str, value: str) -> float:
        """Calculate extraction confidence based on pattern match quality."""
        # Direct dictionary matches have higher confidence
        if value.lower() in self.entity_values.get(entity_type, set()):
            return 0.95
        return 0.75


class IntentClassifier:
    """
    Intent classifier using pattern matching and semantic features.

    Production systems would use trained classifiers, but pattern-based
    classification provides interpretable and modifiable rules.
    """

    # Intent patterns with associated confidence weights
    INTENT_PATTERNS = {
        Intent.GREETING: [
            (r"\b(hi|hello|hey|greetings|good morning|good afternoon|good evening)\b", 0.9),
            (r"\bhow are you\b", 0.7),
        ],
        Intent.FAREWELL: [
            (r"\b(bye|goodbye|see you|farewell|later)\b", 0.95),
            (r"\b(thanks|thank you)\s+(for|bye|now)\b", 0.6),
        ],
        Intent.THANK: [
            (r"\b(thank|thanks|appreciate)\b", 0.9),
            (r"\bthat('s| is) (very|really|so) (helpful|nice|great)\b", 0.8),
        ],
        Intent.APOLOGIZE: [
            (r"\b(sorry|apologize|excuse me|pardon)\b", 0.95),
        ],
        Intent.COMMAND: [
            (r"\b(go|move|walk|stop|turn|grab|pick)\b", 0.7),
            (r"\bdo (something|that)\b", 0.5),
        ],
        Intent.REQUEST: [
            (r"\b(please|could you|would you|can you)\b", 0.85),
            (r"\b(i would like|i want|i need)\b", 0.7),
        ],
        Intent.QUESTION: [
            (r"\b(what|who|where|when|why|how)\b", 0.9),
            (r"\b(can you tell me|do you know)\b", 0.75),
        ],
        Intent.INFORM: [
            (r"\b(it is|they are|there is)\b", 0.6),
            (r"\b(the|this|that) (is|are)\b", 0.5),
        ],
        Intent.CLARIFY: [
            (r"\b(what do you mean|i don('t| not) understand)\b", 0.9),
            (r"\b(can you repeat|say that again)\b", 0.85),
            (r"\b(which one|what( )?(kind|type))\b", 0.7),
        ],
        Intent.CONFIRM: [
            (r"\b(yes|correct|right|that('s| is) (right|correct))\b", 0.9),
            (r"\bi (agree|see|understand)\b", 0.7),
        ],
        Intent.DENY: [
            (r"\b(no|not|correct|wrong)\b", 0.8),
        ],
    }

    def classify(self, text: str) -> Tuple[Intent, float]:
        """
        Classify the intent of user text.

        Returns:
            Tuple of (intent, confidence_score)
        """
        text_lower = text.lower()
        best_intent = Intent.UNKNOWN
        best_confidence = 0.0

        for intent, patterns in self.INTENT_PATTERNS.items():
            for pattern, base_confidence in patterns:
                if re.search(pattern, text_lower):
                    # Adjust confidence based on match specifics
                    confidence = base_confidence

                    # Boost if pattern matches full text
                    if re.fullmatch(pattern, text_lower):
                        confidence = min(confidence * 1.1, 1.0)

                    if confidence > best_confidence:
                        best_confidence = confidence
                        best_intent = intent

        return best_intent, best_confidence


class ReferenceResolver:
    """
    Anaphoric reference resolution for pronouns and descriptions.

    Tracks discourse entities and resolves references across
    conversation turns.
    """

    PRONOUN_PATTERNS = {
        "it": ["object"],
        "they": ["object", "person"],
        "them": ["object", "person"],
        "he": ["person"],
        "she": ["person"],
        "him": ["person"],
        "her": ["person", "object"],
        "this": ["object", "location"],
        "that": ["object", "location"],
    }

    DEMONSTRATIVE_PATTERN = r"\b(this|that|these|those)\b\s*(one|ones)?"

    def __init__(self):
        self.mention_history: List[Dict[str, Any]] = []
        self.current_entities: Dict[str, Dict[str, Any]] = {}

    def resolve(self, text: str, entities: List[Entity]) -> List[Reference]:
        """
        Resolve references in the current utterance.

        Args:
            text: Current utterance text
            entities: Entities extracted from current utterance

        Returns:
            List of resolved references
        """
        references = []

        # Resolve pronouns
        for match in re.finditer(r"\b(it|they|them|he|she|him|her|this|that)\b", text.lower()):
            reference_text = match.group()
            pos = (match.start(), match.end())

            # Get possible referent types
            referent_types = self.PRONOUN_PATTERNS.get(reference_text, ["object"])

            # Find best antecedent
            antecedent = self._find_antecedent(referent_types, entities)

            if antecedent:
                references.append(Reference(
                    text=reference_text,
                    position=pos,
                    referent_type=referent_types[0],
                    antecedent=antecedent,
                    confidence=0.85
                ))

        # Update mention history
        for entity in entities:
            self.mention_history.append({
                "type": entity.type,
                "value": entity.value,
                "text": entity.raw_text,
                "timestamp": datetime.now().isoformat()
            })

        # Keep only recent mentions
        max_history = 10
        if len(self.mention_history) > max_history:
            self.mention_history = self.mention_history[-max_history:]

        return references

    def _find_antecedent(
        self,
        referent_types: List[str],
        current_entities: List[Entity]
    ) -> Optional[str]:
        """Find the most likely antecedent for a reference."""
        # First check current utterance for immediate reference
        for entity in current_entities:
            if entity.type in referent_types:
                return entity.value

        # Then check recent history
        for mention in reversed(self.mention_history):
            if mention["type"] in referent_types:
                return mention["value"]

        return None


class NaturalLanguageUnderstanding:
    """
    Complete NLU pipeline combining all components.

    Integrates intent classification, entity extraction, and reference
    resolution into a unified parsing interface.
    """

    def __init__(self):
        self.intent_classifier = IntentClassifier()
        self.entity_extractor = EntityExtractor()
        self.reference_resolver = ReferenceResolver()

    def parse(self, text: str) -> NLUParsing:
        """
        Complete NLU parsing of user input.

        Args:
            text: Raw user input text

        Returns:
            Complete NLUParsing with all analyzed components
        """
        # Extract entities
        entities = self.entity_extractor.extract(text)

        # Classify intent
        intent, intent_confidence = self.intent_classifier.classify(text)

        # Resolve references
        references = self.reference_resolver.resolve(text, entities)

        # Calculate additional features
        sentiment = self._analyze_sentiment(text)
        urgency = self._analyze_urgency(text)
        politeness = self._analyze_politeness(text)

        return NLUParsing(
            text=text,
            intent=intent,
            confidence=intent_confidence,
            entities=entities,
            references=references,
            sentiment=sentiment,
            urgency=urgency,
            politeness=politeness
        )

    def _analyze_sentiment(self, text: str) -> float:
        """Analyze sentiment polarity of input (-1 to 1)."""
        positive_words = {"good", "great", "nice", "wonderful", "excellent", "happy", "thanks"}
        negative_words = {"bad", "terrible", "awful", "horrible", "sorry", "wrong", "hate"}

        text_lower = text.lower()
        score = 0.0

        for word in positive_words:
            if word in text_lower:
                score += 0.2
        for word in negative_words:
            if word in text_lower:
                score -= 0.2

        return max(-1.0, min(1.0, score))

    def _analyze_urgency(self, text: str) -> float:
        """Analyze urgency of input (0 to 1)."""
        urgency_indicators = {
            "now": 0.8,
            "immediately": 0.9,
            "urgent": 0.7,
            "asap": 0.85,
            "quickly": 0.6,
            "hurry": 0.8,
            "fast": 0.5,
            "right away": 0.85
        }

        text_lower = text.lower()
        urgency = 0.1  # Base urgency

        for indicator, level in urgency_indicators.items():
            if indicator in text_lower:
                urgency = max(urgency, level)

        # Questions are typically less urgent
        if text.strip().endswith("?"):
            urgency *= 0.7

        return urgency

    def _analyze_politeness(self, text: str) -> float:
        """Analyze politeness markers in input (0 to 1)."""
        polite_markers = {
            "please": 0.8,
            "thank you": 0.9,
            "thanks": 0.7,
            "would you": 0.75,
            "could you": 0.75,
            "would appreciate": 0.85,
            "if you could": 0.7
        }

        text_lower = text.lower()
        politeness = 0.5  # Base politeness

        for marker, level in polite_markers.items():
            if marker in text_lower:
                politeness = max(politeness, level)

        return politeness


# Demonstration of NLU pipeline
def demonstrate_nlu():
    """Demonstrate the NLU pipeline with example inputs."""

    print("=== Natural Language Understanding Demo ===\n")

    # Initialize NLU
    nlu = NaturalLanguageUnderstanding()

    # Example inputs
    test_inputs = [
        "Hello Fubuni, how are you today?",
        "Can you please bring me the red cup from the kitchen?",
        "What is on the table?",
        "That one, not this one.",
        "Thank you so much, you're really helpful!",
        "I don't understand what you mean.",
        "Stop moving and wait there."
    ]

    for user_input in test_inputs:
        print(f"Input: \"{user_input}\"")

        result = nlu.parse(user_input)

        print(f"  Intent: {result.intent.value} (confidence: {result.confidence:.2f})")
        print(f"  Sentiment: {result.sentiment:.2f}, Urgency: {result.urgency:.2f}, Politeness: {result.politeness:.2f}")

        if result.entities:
            entity_strs = [f"{e.type}:'{e.value}'" for e in result.entities]
            print(f"  Entities: {', '.join(entity_strs)}")

        if result.references:
            ref_strs = [f"'{r.text}'->{r.antecedent}" for r in result.references]
            print(f"  References: {', '.join(ref_strs)}")

        print()


if __name__ == "__main__":
    demonstrate_nlu()
```

## 6.5 Multi-Modal Interaction

Humanoid robot interaction naturally extends beyond speech to encompass gesture, facial expression, eye gaze, and physical action. Multi-modal interaction systems integrate these channels to create more natural and expressive communication. For humanoid robots, the alignment of verbal and non-verbal cues is essential for coherent social interaction.

Gesture recognition enables robots to understand and respond to pointing, beckoning, waving, and other communicative gestures. Computer vision systems track hand positions and recognize predefined gesture vocabularies. When combined with speech, gestures provide disambiguation that resolves ambiguity in spoken commands. A user pointing at an object while saying "Bring me that" provides clear reference where speech alone would be ambiguous.

Facial expression recognition and generation create emotional feedback loops in human-robot interaction. Robots that can recognize user emotions through facial expressions can adapt their responses accordingly. Similarly, robots that display appropriate facial expressions communicate their internal state and create more engaging interactions. Expressive eyes, brows, and mouth movements supplement speech with emotional content.

Eye gaze direction communicates attention and intention in social interaction. Humans naturally follow the gaze of their conversation partners to understand what they are attending to. Humanoid robots can use eye tracking to understand user attention and guide their own gaze to support joint attention behaviors. Gaze aversion during thinking or listening mimics human social patterns and makes robot behavior feel more natural.

The following example implements a multi-modal integration layer that coordinates speech, gesture, and facial expression:

```python
#!/usr/bin/env python3
"""
Multi-Modal Interaction System for Humanoid Robots

Integrates speech, gesture, facial expression, and gaze for natural
human-robot communication across multiple channels.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple
from abc import ABC, abstractmethod
import asyncio


class Modality(Enum):
    """Enumeration of interaction modalities."""
    SPEECH = "speech"
    GESTURE = "gesture"
    FACIAL_EXPRESSION = "facial"
    EYE_GAZE = "gaze"
    BODY_POSTURE = "posture"
    TOUCH = "touch"


class GestureType(Enum):
    """Predefined gesture types for robot execution."""
    POINT_LEFT = "point_left"
    POINT_RIGHT = "point_right"
    POINT_UP = "point_up"
    POINT_DOWN = "point_down"
    WAVE = "wave"
    NOD = "nod"
    SHAKE = "shake"
    THUMBS_UP = "thumbs_up"
    OPEN_HAND = "open_hand"
    CLOSED_FIST = "closed_fist"
    SHRUG = "shrug"
    beckon = "beckon"


class FacialExpression(Enum):
    """Facial expressions for robot display."""
    NEUTRAL = "neutral"
    HAPPY = "happy"
    SAD = "sad"
    SURPRISED = "surprised"
    FEARFUL = "fearful"
    ANGRY = "angry"
    DISGUSTED = "disgusted"
    CONTEMPLATIVE = "contemplative"
    CONFUSED = "confused"
    SMILING = "smiling"
    LISTENING = "listening"
    THINKING = "thinking"


class GazeTarget(Enum):
    """Gaze targets for eye movement."""
    SPEAKER = "speaker"
    OBJECT = "object"
    LOCATION = "location"
    DOWN = "down"  # Submissive/respectful
    UP = "up"      # Thinking/reflective
    AWAY = "away"  # Averting gaze
    NEUTRAL = "neutral"


@dataclass
class MultiModalMessage:
    """
    Multi-modal message with synchronized expression across channels.

    Coordinates speech, gesture, facial expression, and gaze into
    a coherent communication package.
    """
    text: str
    gesture: Optional[GestureType] = None
    facial_expression: FacialExpression = FacialExpression.NEUTRAL
    gaze_target: GazeTarget = GazeTarget.SPEAKER
    emotion_intensity: float = 0.5  # 0.0 to 1.0
    duration: float = 2.0  # Seconds for gesture/expression display
    priority: int = 0  # Higher priority interrupts lower
    timestamp: float = field(default_factory=lambda: __import__('time').time())


class GestureController:
    """
    Controller for robot gesture execution.

    Manages gesture sequencing, timing, and coordination with
    other interaction modalities.
    """

    def __init__(self):
        self.current_gesture: Optional[GestureType] = None
        self.gesture_queue: List[MultiModalMessage] = []
        self.is_executing = False

    async def execute_gesture(
        self,
        gesture: GestureType,
        duration: float = 1.0,
        repeat: int = 1
    ) -> bool:
        """
        Execute a gesture with specified duration and repetition.

        Args:
            gesture: The gesture to execute
            duration: Duration of each gesture cycle in seconds
            repeat: Number of times to repeat the gesture

        Returns:
            True if gesture executed successfully
        """
        self.current_gesture = gesture
        self.is_executing = True

        for i in range(repeat):
            print(f"[Gesture] Executing: {gesture.value}")
            await asyncio.sleep(duration)

            # Check for interruption
            if not self.is_executing:
                return False

        self.is_executing = False
        self.current_gesture = None
        return True

    def queue_gesture(self, message: MultiModalMessage) -> None:
        """Queue a multi-modal message for gesture execution."""
        self.gesture_queue.append(message)

    async def process_queue(self) -> None:
        """Process queued gestures in priority order."""
        while self.gesture_queue:
            # Sort by priority
            self.gesture_queue.sort(key=lambda m: m.priority, reverse=True)

            message = self.gesture_queue.pop(0)

            if message.gesture:
                await self.execute_gesture(
                    message.gesture,
                    duration=message.duration
                )


class FacialExpressionController:
    """
    Controller for robot facial expression display.

    Manages expression transitions, blends, and timing for
    emotionally expressive robot faces.
    """

    def __init__(self):
        self.current_expression: FacialExpression = FacialExpression.NEUTRAL
        self.expression_intensity: float = 0.5
        self.expression_history: List[Dict[str, Any]] = []

    def set_expression(
        self,
        expression: FacialExpression,
        intensity: float = 0.5,
        duration: Optional[float] = None
    ) -> None:
        """
        Set the current facial expression.

        Args:
            expression: The expression to display
            intensity: Expression intensity (0.0 to 1.0)
            duration: Optional duration before returning to neutral
        """
        self.current_expression = expression
        self.expression_intensity = intensity

        self.expression_history.append({
            "expression": expression.value,
            "intensity": intensity,
            "timestamp": __import__('time').time()
        })

        print(f"[Face] Expression: {expression.value} (intensity: {intensity:.2f})")

        if duration:
            asyncio.create_task(self._reset_after_duration(duration))

    async def _reset_after_duration(self, duration: float) -> None:
        """Reset to neutral after duration expires."""
        await asyncio.sleep(duration)
        self.set_expression(FacialExpression.NEUTRAL, 0.0)

    def get_emotion_for_expression(
        self,
        expression: FacialExpression
    ) -> Dict[str, float]:
        """Get emotion scores corresponding to a facial expression."""
        emotion_maps = {
            FacialExpression.HAPPY: {"joy": 0.9, "sadness": 0.0, "anger": 0.0},
            FacialExpression.SAD: {"joy": 0.0, "sadness": 0.8, "anger": 0.1},
            FacialExpression.SURPRISED: {"joy": 0.4, "sadness": 0.0, "anger": 0.0, "fear": 0.6},
            FacialExpression.ANGRY: {"joy": 0.0, "sadness": 0.1, "anger": 0.9},
            FacialExpression.FEARFUL: {"joy": 0.0, "sadness": 0.3, "fear": 0.8},
            FacialExpression.CONTEMPLATIVE: {"joy": 0.3, "sadness": 0.1, "anger": 0.0},
            FacialExpression.CONFUSED: {"joy": 0.1, "sadness": 0.2, "fear": 0.3},
            FacialExpression.LISTENING: {"joy": 0.3, "sadness": 0.0, "anger": 0.0},
            FacialExpression.THINKING: {"joy": 0.2, "sadness": 0.1, "anger": 0.0},
            FacialExpression.SMILING: {"joy": 0.7, "sadness": 0.0, "anger": 0.0},
            FacialExpression.NEUTRAL: {"joy": 0.2, "sadness": 0.1, "anger": 0.0},
        }
        return emotion_maps.get(expression, {"joy": 0.2, "sadness": 0.1, "anger": 0.0})


class GazeController:
    """
    Controller for robot eye gaze direction.

    Manages eye movements for natural social gaze behaviors
    including joint attention and conversational gaze patterns.
    """

    def __init__(self):
        self.current_target: GazeTarget = GazeTarget.NEUTRAL
        self.gaze_position: Tuple[float, float] = (0.0, 0.0)  # x, y angles
        self.is_tracking = False
        self.tracking_target: Optional[str] = None

    def set_gaze(
        self,
        target: GazeTarget,
        position: Optional[Tuple[float, float]] = None,
        smooth: bool = True
    ) -> None:
        """
        Set gaze direction.

        Args:
            target: The gaze target type
            position: Specific position for object/location gazes
            smooth: Whether to use smooth eye movement
        """
        self.current_target = target

        if position:
            self.gaze_position = position

        gaze_angles = self._target_to_angles(target, position)
        print(f"[Gaze] Target: {target.value}, Angles: {gaze_angles}")

    def _target_to_angles(
        self,
        target: GazeTarget,
        position: Optional[Tuple[float, float]]
    ) -> Tuple[float, float]:
        """Convert gaze target to eye angles."""
        angle_map = {
            GazeTarget.SPEAKER: (0.0, 5.0),    # Center, slightly up
            GazeTarget.NEUTRAL: (0.0, 0.0),    # Center
            GazeTarget.DOWN: (0.0, -15.0),     # Looking down
            GazeTarget.UP: (0.0, 15.0),        # Looking up
            GazeTarget.AWAY: (30.0, 0.0),      # Looking away
        }

        if target == GazeTarget.OBJECT or target == GazeTarget.LOCATION:
            return position if position else (0.0, 0.0)

        return angle_map.get(target, (0.0, 0.0))

    async def gaze_at_person(self, person_position: Tuple[float, float]) -> None:
        """Track a specific person's position with gaze."""
        self.is_tracking = True
        self.tracking_target = "person"

        # Simulate tracking with updates
        while self.is_tracking:
            self.gaze_position = person_position
            await asyncio.sleep(0.1)

    def stop_tracking(self) -> None:
        """Stop gaze tracking and return to neutral."""
        self.is_tracking = False
        self.tracking_target = None
        self.set_gaze(GazeTarget.NEUTRAL)


class MultiModalIntegrator:
    """
    Main integrator for multi-modal robot interaction.

    Coordinates speech, gesture, facial expression, and gaze
    into coherent, contextually appropriate responses.
    """

    def __init__(self):
        self.gesture_controller = GestureController()
        self.face_controller = FacialExpressionController()
        self.gaze_controller = GazeController()
        self.modality_enabled = {
            Modality.SPEECH: True,
            Modality.GESTURE: True,
            Modality.FACIAL_EXPRESSION: True,
            Modality.EYE_GAZE: True,
            Modality.BODY_POSTURE: True,
            Modality.TOUCH: True
        }

    def enable_modality(self, modality: Modality, enabled: bool) -> None:
        """Enable or disable a specific modality."""
        self.modality_enabled[modality] = enabled

    async def express(
        self,
        message: MultiModalMessage,
        async_speech: bool = True
    ) -> None:
        """
        Express a multi-modal message across all enabled channels.

        Coordinates the timing and execution of expressions across
        gesture, face, and gaze modalities.
        """
        tasks = []

        # Gesture
        if message.gesture and self.modality_enabled[Modality.GESTURE]:
            tasks.append(self.gesture_controller.execute_gesture(
                message.gesture,
                duration=message.duration
            ))

        # Facial expression
        if self.modality_enabled[Modality.FACIAL_EXPRESSION]:
            self.face_controller.set_expression(
                message.facial_expression,
                intensity=message.emotion_intensity,
                duration=message.duration
            )

        # Gaze
        if self.modality_enabled[Modality.EYE_GAZE]:
            self.gaze_controller.set_gaze(message.gaze_target)

        # Execute all concurrently
        if tasks:
            await asyncio.gather(*tasks)

        # Return to neutral after duration
        await asyncio.sleep(message.duration * 0.5)
        self.face_controller.set_expression(FacialExpression.NEUTRAL, 0.0)
        self.gaze_controller.set_gaze(GazeTarget.NEUTRAL)

    def create_happy_response(self, text: str) -> MultiModalMessage:
        """Create a happy multi-modal response."""
        return MultiModalMessage(
            text=text,
            facial_expression=FacialExpression.SMILING,
            gaze_target=GazeTarget.SPEAKER,
            emotion_intensity=0.7,
            gesture=GestureType.NOD
        )

    def create_thinking_response(self, text: str) -> MultiModalMessage:
        """Create a contemplative multi-modal response."""
        return MultiModalMessage(
            text=text,
            facial_expression=FacialExpression.THINKING,
            gaze_target=GazeTarget.UP,
            emotion_intensity=0.4
        )

    def create_confused_response(self, text: str) -> MultiModalMessage:
        """Create a confused multi-modal response for clarification."""
        return MultiModalMessage(
            text=text,
            facial_expression=FacialExpression.CONFUSED,
            gaze_target=GazeTarget.AWAY,
            emotion_intensity=0.5,
            gesture=GestureType.SHRUG
        )

    def create_pointing_response(
        self,
        text: str,
        direction: str
    ) -> MultiModalMessage:
        """Create a response with pointing gesture."""
        gesture_map = {
            "left": GestureType.POINT_LEFT,
            "right": GestureType.POINT_RIGHT,
            "up": GestureType.POINT_UP,
            "down": GestureType.POINT_DOWN
        }

        return MultiModalMessage(
            text=text,
            gesture=gesture_map.get(direction, GestureType.POINT_RIGHT),
            facial_expression=FacialExpression.NEUTRAL,
            gaze_target=GazeTarget.OBJECT,
            emotion_intensity=0.5
        )


# Demonstration of multi-modal interaction
async def demonstrate_multi_modal():
    """Demonstrate multi-modal interaction system."""

    print("=== Multi-Modal Interaction Demo ===\n")

    # Initialize integrator
    integrator = MultiModalIntegrator()

    # Example multi-modal messages
    messages = [
        MultiModalMessage(
            text="Hello! I'm happy to see you!",
            facial_expression=FacialExpression.SMILING,
            gesture=GestureType.NOD,
            gaze_target=GazeTarget.SPEAKER,
            emotion_intensity=0.8
        ),
        MultiModalMessage(
            text="Let me think about that...",
            facial_expression=FacialExpression.THINKING,
            gaze_target=GazeTarget.UP,
            emotion_intensity=0.5
        ),
        MultiModalMessage(
            text="I'm not sure which one you mean.",
            facial_expression=FacialExpression.CONFUSED,
            gesture=GestureType.SHRUG,
            gaze_target=GazeTarget.AWAY,
            emotion_intensity=0.6
        ),
        MultiModalMessage(
            text="The cup is over there on the table.",
            facial_expression=FacialExpression.NEUTRAL,
            gesture=GestureType.POINT_RIGHT,
            gaze_target=GazeTarget.OBJECT,
            emotion_intensity=0.4
        ),
        MultiModalMessage(
            text="Great! I understand now!",
            facial_expression=FacialExpression.HAPPY,
            gesture=GestureType.THUMBS_UP,
            gaze_target=GazeTarget.SPEAKER,
            emotion_intensity=0.9
        )
    ]

    print("Executing multi-modal expressions:\n")

    for i, message in enumerate(messages, 1):
        print(f"--- Expression {i} ---")
        print(f"Text: \"{message.text}\"")
        print(f"Face: {message.facial_expression.value}")
        if message.gesture:
            print(f"Gesture: {message.gesture.value}")
        print(f"Gaze: {message.gaze_target.value}")
        print()

        await integrator.express(message)
        await asyncio.sleep(0.5)

    print("\nMulti-modal demonstration complete.")


if __name__ == "__main__":
    asyncio.run(demonstrate_multi_modal())
```

## Chapter Summary

This chapter explored the comprehensive topic of conversational AI for humanoid robots, covering the essential technologies and integration patterns that enable natural human-robot dialogue.

1. **Conversational AI Foundations**: The chapter introduced the conversational AI pipeline for robotics, explaining how speech recognition, natural language understanding, dialogue management, and speech synthesis work together to create natural interaction experiences.

2. **GPT Integration**: We examined patterns for integrating large language models like GPT into robot dialogue systems, including context management, prompt engineering, and hybrid architectures that combine cloud and local processing.

3. **Speech Recognition and Synthesis**: The chapter covered the practical implementation of speech interfaces including audio preprocessing, voice activity detection, and text-to-speech synthesis for robot output.

4. **Natural Language Understanding**: We implemented comprehensive NLU capabilities including intent classification, entity extraction, reference resolution, and sentiment analysis for understanding user input.

5. **Multi-Modal Interaction**: The chapter explored how speech integrates with gesture, facial expression, and eye gaze to create coherent, expressive robot communication across multiple channels.

### Key Concepts

- **Conversational Pipeline**: The sequence of ASR, NLU, dialogue management, NLG, and TTS that enables voice interaction
- **Intent Classification**: Assigning user utterances to predefined categories for appropriate response selection
- **Entity Extraction**: Identifying specific objects, locations, and other relevant information in user speech
- **Multi-Modal Integration**: Coordinating verbal and non-verbal communication channels for natural expression
- **Context Management**: Maintaining conversation state across extended interactions

### Key Terminology

- **ASR (Automatic Speech Recognition)**: Technology for converting spoken language to text
- **TTS (Text-to-Speech)**: Technology for converting text to spoken language
- **NLU (Natural Language Understanding)**: Processing language to extract meaning and intent
- **Dialogue Manager**: System component that maintains conversation state and coordinates responses
- **Multi-Modal Interaction**: Using multiple communication channels simultaneously

### Further Reading

- **Speech and Language Processing**: Jurafsky & Martin - Comprehensive NLP textbook
- **OpenAI Whisper Documentation**: State-of-the-art speech recognition
- **ROS 2 Audio Packages**: Audio processing and ASR integration for ROS
- **Human-Robot Interaction Research**: Contemporary papers on social robotics

### Next Chapter

Chapter 7 explores **Robot Learning and Adaptation**, covering how robots learn from experience, adapt to new situations, and improve their capabilities over time through machine learning techniques.

---

**Part 6: Conversational Robotics** | [Chapter 6: Conversational Robotics](part-6-conversational/conversational-robotics) | [Appendix: Assessment Rubrics](appendix/D-assessment-rubrics)
