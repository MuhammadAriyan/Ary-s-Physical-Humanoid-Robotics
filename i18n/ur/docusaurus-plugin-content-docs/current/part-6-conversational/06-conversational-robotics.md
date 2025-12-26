---
title: "مکالمہ روبوٹکس"
sidebar_position: 6
---

# Chapter 6: مکالمہ روبوٹکس (Conversational Robotics)

## سیکھنے کے مقاصد

اس باب کے اختتام پر، آپ درج ذیل کر سکیں گے:
- مکالمہ AI (Conversational AI) کے اصولوں اور ہیومینوئیڈ روبوٹس میں اس کے اطلاق کو سمجھنا
- قدرتی مکالمے کی پیداوار کے لیے GPT جیسے بڑے زبان کے ماڈلز (Large language models) کو مربوط کرنا
- آواز کے تعامل (Voice interaction) کے لیے تقریر کی پہچان (Speech recognition) اور متن سے تقریر (Text-to-speech) کے پائپ لائنز کو نافذ کرنا
- صارف کے ارادے کو سمجھنے کے لیے قدرتی زبان کی سمجھ (Natural language understanding) کے نظاموں کو ڈیزائن کرنا
- تقریر، اشارے، اور وژن کو ملا کر کثیرالطریقہ (Multi-modal) تعامل کے نظاموں کو بنانا
- مکمل مکالمے کے نظام بنانا جو معنی خیز انسان روبوٹ مکالمے کو ممکن بنائے

## 6.1 روبوٹکس میں مکالمہ AI کا تعارف

مکالمہ AI (Conversational AI) انسانوں کے روبوٹس کے ساتھ تعامل کرنے کے طریقے میں ایک بنیادی تبدیلی کی نمائندگی کرتا ہے۔ جoyssticks، بٹنوں، یا پیچیدہ پروگرامنگ انٹرفیس پر انحصار کرنے کے بجائے، مکالمہ AI قدرتی بولی ہوئی گفتگو کو اہم تعامل کے ذریعے کے طور پر ممکن بناتا ہے۔ ہیومینوئیڈ روبوٹس جو انسانی ماحول میں کام کرنے کے لیے ڈیزائن کیے گئے ہیں، گھریلو، صحت کی دیکھبھال، اور خدماتی ترتیبات میں ہموار انضمام کے لیے یہ اہلیت ضروری ہے۔

کمانڈ پر مبنی انٹرفیس سے مکالمہ تعامل کی ترقی وسیع تر انسان کمپیوٹر تعامل کے رجحانات کی عکاسی کرتی ہے۔ ابتدائی روبوٹک نظاموں کو صارفین کو مخصوص کمانڈ نحو یا بٹن کی مخصوص کیمیئر سیکھنی پڑتی تھی۔ جدید مکالمہ انٹرفیس قدرتی زبان کو قبول کرتے ہیں، صارفین کو خصوصی تربیت کے بغیر اپنے ارادوں کو بیان کرنے کی اجازت دیتے ہیں۔ روبوٹ کنٹرول کی یہ جمہوریانہ سازی ہیومینوئیڈ روبوٹکس کو بوڑھے صارفین، بچوں، اور تکنیکی پس منظر کے بغیر افراد کے لیے کھولتی ہے۔

ہیومینوئیڈ روبوٹس اپنی انسان نما شکل کی وجہ سے مکالمہ AI (Conversational AI) کے لیے منفرد مواقع پیش کرتے ہیں۔ جب ایک روبوٹ قدرتی طور پر بول سکتا ہے، جذباتی انداز میں اشارے کر سکتا ہے، اور مکالمے کے دوران آنکھوں کا رابطہ برقرار رکھ سکتا ہے، تو تعامل زیادہ فطری اور دلچسپ محسوس ہوتا ہے۔ ایک ہیومینوئیڈ روبوٹ جو پوچھے "کیا آپ مجھ سے ایک کپ چائے لاوانے چاہتے ہیں؟" ایک سکرین پر مبنی اسسٹنٹ سے کہیں زیادہ فطری ہے جو وہی سوال پیش کرتا ہے۔ تعامل کی تجسم شدہ نوعیت سماجی ذہانت کی توقعات پیدا کرتی ہے جو مکالمہ AI (Conversational AI) کو پورا کرنا چاہیے۔

مکالمہ روبوٹکس کی تکنیکی چیلنجز سادے تقریر کی پراسیسنگ سے آگے بڑھتے ہیں۔ ایک مکالمہ ہیومینوئیڈ روبوٹ کو توسیع تعاملات میں سیاق و سباق برقرار رکھنا چاہیے، ماحول میں موجود اشیاء اور افراد کے مبہم حوالوں کو سمجھنا چاہیے، روبوٹ کی شخصیت اور اہلیتوں کے مطابق مناسب جوابات تیار کرنا چاہیے، اور لفظی آؤٹ پٹ کو چہرے کے ظہور اور اشاروں جیسے غیر لفظی اشاروں کے ساتھ ہم وقت کرنا چاہیے۔ ان ضروریات کو ایک AI ذیلی نظاموں کی وسیع رینج کی انضمام کی ضرورت ہے بشمول تقریر کی پہچان، قدرتی زبان کی سمجھ، مکالمہ کا انتظام، قدرتی زبان کی پیداوار، اور تقریر کی ترکیب۔

### مکالمہ AI پائپ لائن (The Conversational AI Pipeline)

روبوٹس کے لیے مکالمہ AI (Conversational AI) ایک پیچیدہ پراسیسنگ مراحل کی پائپ لائن پر مشتمل ہے، ہر ایک پچھلے مرحلے کی پیداوار پر تعمیر کرتا ہے۔ مضبوط مکالمہ نظاموں کو ڈیزائن کرنے کے لیے یہ پائپ لائن سمجھنا ضروری ہے جو حقیقت دنیا کی گفتگو کی تغیر کو سنبھال سکیں۔

پائپ لائن **خودکار تقریر کی پہچان** (Automatic Speech Recognition - ASR) سے شروع ہوتا ہے، جو خام آڈیو ان پٹ کو متن رونویشیوں میں تبدیل کرتا ہے۔ جدید ASR نظام گہرے عصبی نیٹ ورکس کا استعمال کرتے ہیں جو وسیع تقریر کورپرا پر تربیت یافتہ ہیں تاکہ متنوع اسپیکروں اور آکوسٹک حالات میں زیادہ درستگی حاصل کی جا سکے۔ روبوٹ ایپلیکیشنز کے لیے، ASR کو پس منظر کی شور، متعدد اسپیکروں، اور روبوٹ کے جسمانی ماحول کی آکوسٹک خاصیتوں کو سنبھالنا ہوگا۔

تقریر کی پہچان کے بعد، **قدرتی زبان کی سمجھ** (Natural Language Understanding - NLU) رونویشی ٹیکسٹ کی معنی کی تشریح کرتی ہے۔ NLU صارف کے ارادے کی نشاندہی کرتی ھے، متعلقہ ہستیوں کو نکالتی ہے، اور سیاق و سباق میں اشیاء یا افراد کے حوالوں کو حل کرتی ہے۔ "مجھے رکھنے سے لے آؤ" جیسے کمانڈ کے لیے، NLU ارادے کو اشیاء کی فراہمی کی درخواست کے طور پر شناخا کرتی ہے، ہستی کی قسم (کپ)، رنگ (سرخ)، اور مقام (رکھنے) نکالتی ہے۔

**مکالمہ کا انتظام** (Dialogue management) مکالمہ حالت برقرار رکھتا ہے اور مناسب نظام جوابات کا تعین کرتا ہے۔ یہ ٹریک کرتا ہے کہ کیا کہا گیا، موجودہ موضوع کیا ہے، اور کون سی معلومات قائم ہوئی ہیں۔ مکالمہ کے منتظم وضاحت طلب کرنے، سمجھنے کی تصدیق کرنے، یا معلومات فراہم کرنے جیسے مکالمے کے افعال کو بھی سنبھالتا ہے۔

**قدرتی زبان کی پیداوار** (Natural Language Generation - NLG) متنی جوابات تیار کرتا ہے جو نظام بولے گا۔ جدید طریقے NLG کے لیے بڑے زبان کے ماڈلز کا استعمال کرتے ہیں، جو مکالمہ سیاق و سباق کے مطابق fluent متن تیار کرتے ہیں اور مستقل persona برقرار رکھتے ہیں۔ NLG کو روبوٹ کی شخصیت اور مختلف جوابات کے انداز کی مناسبیت پر بھی غور کرنا چاہیے۔

آخر میں، **متن سے تقریر** (Text-to-speech - TTS) ترکیب تیار کردہ متن کو قابل سن آؤٹ پٹ میں تبدیل کرتی ہے۔ جدید TTS نظام جو عصبی نیٹ ورک فن تعمیرات کا استعمال کرتے ہیں، قابل ستائش طور پر قدرتی آواز والی تقریر پیدا کرتے ہیں جس میں مناسب لےج اور جذباتی لہجہ ہوتا ہے۔ TTS آؤٹ پٹ کو روبوٹ کے ہونٹوں کی حرکات اور چہرے کے ظہور کے ساتھ قدرتی تجسم کے لیے ہم وقت بھی کرنا چاہیے۔

### مزید دیکھیں

- **حصہ 1**: [Physical AI کا تعارف](part-1-foundations/introduction-to-physical-ai) بنیادی تصورات کا احاطہ کرتا ہے
- **حصہ 3**: [Gazebo اور Unity Simulation](part-3-simulation/gazebo-unity-simulation) پرسپیشن کے لیے سینسر کی تریبیح پر بحث کرتا ہے
- **حصہ 5**: [ہیومینوئیڈ ترقی](part-5-humanoid/humanoid-robot-development) ہیومینوئیڈ روبوٹ ڈیزائن کا احاطہ کرتا ہے

## 6.2 قدرتی مکالمے کے لیے GPT ماڈلز کا انضمام

بڑے زبان کے ماڈلز نے مکالمہ AI کو قدرتی زبان کی سمجھ اور پیداوار میں پہلے سے نہیں دیکھی گئی اہلیتوں فراہم کرکے انقلاب برپا کردیا ہے۔ GPT خاندان کے ماڈلز، جو OpenAI اور دیگر نے تیار کیے، وسیع موضوعات میں连贯، سیاق و سباق کے مطابق مکالمے میں مصروف ہو سکتے ہیں۔ ان ماڈلز کو روبوٹ نظاموں میں مربوط کرنا قدرتی گفتگو کو ممکن بناتا ہے جبکہ مناسب حفاظتی پابندیوں اور کام کے فوکس کو برقرار رکھتا ہے۔

GPT ماڈلز کو روبوٹ مکالمہ نظاموں میں مربوط کرنے کے لیے احتیاط سے فن تعمیر ڈیزائن کی ضرورت ہے۔ کلاؤڈ پر مبنی زبان کے ماڈلز سے براہ راست رابطہ اتنی تاخیر پیدا کرتا ہے کہ مکالمہ غیر فطری محسوس ہو سکتا ہے۔ نیٹ ورک انحصار بھی حفاظت کے لیے اہم ایپلیکیشنز کے لیے قابلیت کی قابل اعتمادی کے بارے میں خدشات اٹھاتے ہیں۔ کامیاب انضمام کی حکمت عملی عام طور پر ہائبرڈ طریقہ شامل کرتی ہے جہاں ہلکے مقامی ماڈلز روزانہ تعاملات کو سنبھالتے ہیں جبکہ کلاؤڈ ماڈلز جب ضرورت ہو تو خاص KNOWLEDGE فراہم کرتے ہیں۔

مندرجہ ذیل مثال ایک مکالمہ کے منتظم کا مظاہرہ کرتا ہے جو جوابات کی پیداوار کے لیے GPT ماڈل کے ساتھ رابطہ کرتا ہے جبکہ روبوٹ کے مخصوص سیاق و سباق اور حفاظتی پابندیوں کو برقرار رکھتا ہے:

```python
#!/usr/bin/env python3
"""
GPT-Powered Dialogue Manager for Humanoid Robots

یہ ماڈیول ایک مکمل مکالمہ انتظامی نظام فراہم کرتا ہے جو
روبوٹ کے مخصوص سیاق و سباق اور پابندیوں کے ساتھ بڑے زبان کے ماڈلز کو مربوط کرتا ہے۔
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
    """مکالمہ کی ممکنہ حالتوں کی فہرست۔"""
    IDLE = auto()
    LISTENING = auto()
    PROCESSING = auto()
    SPEAKING = auto()
    WAITING_FOR_ACTION = auto()
    ERROR = auto()


class Intent(Enum):
    """روبوٹ کے مخصوص مکالمہ ارادے۔"""
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
    """صارف ان پٹ سے نکالی گئی ہستی کی نمائندگی کرتا ہے۔"""
    type: str
    value: str
    confidence: float
    position: Tuple[int, int]  # شروع اور ختم کریکٹر پوزیشن


@dataclass
class DialogueAct:
    """ارادے اور ہستیوں کے ساتھ مکمل مکالمہ فعل کی نمائندگی کرتا ہے۔"""
    intent: Intent
    entities: List[Entity] = field(default_factory=list)
    confidence: float = 1.0
    raw_text: str = ""
    timestamp: datetime = field(default_factory=datetime.now)


@dataclass
class ConversationContext:
    """ایک مکالمہ سیشن میں سیاق و سباق برقرار رکھتا ہے۔"""
    session_id: str
    user_id: Optional[str]
    robot_name: str = "Fubuni"
    conversation_history: List[Dict[str, Any]] = field(default_factory=list)
    known_entities: Dict[str, Any] = field(default_factory=dict)
    current_topic: Optional[str] = None
    last_action_result: Optional[str] = None
    user_preferences: Dict[str, Any] = field(default_factory=dict)

    def add_exchange(self, user_message: str, robot_response: str) -> None:
        """مکالمہ تاریخ میں صارف-روبوٹ تبادلہ شامل کریں۔"""
        self.conversation_history.append({
            "timestamp": datetime.now().isoformat(),
            "user": user_message,
            "robot": robot_response
        })
        # تاریخ کو کانٹیکسٹ ونڈوز کے لیے قابل انتظام رکھیں
        max_history = 20
        if len(self.conversation_history) > max_history:
            self.conversation_history = self.conversation_history[-max_history:]


class GPTClient:
    """
    GPT پر مبنی زبان کے ماڈلز کے ساتھ رابطے کے لیے کلائنٹ۔

    پرامپٹ تعمیر، API رابطہ، اور جوابات کی تجزیہ کو سنبھالتا ہے
    جبکہ حفاظتی پابندیوں اور سیاق و سباق کے انتظام کو برقرار رکھتا ہے۔
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
        """روبوٹ مکالمے کے لیے ڈیفالٹ سسٹم پرامپٹ تیار کریں۔"""
        return f"""You are Fubuni, a friendly humanoid robot assistant. You are helpful,
polite, and concise in your responses. You assist users with a home
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
        """API درخواست کے لیے پیغام کی فہرست تیار کریں۔"""
        messages = [{"role": "system", "content": self.system_prompt}]

        # سیاق و سباق کے لیے مکالمہ تاریخ شامل کریں
        for exchange in context.conversation_history[-5:]:  # آخری 5 تبادلے
            messages.append({"role": "user", "content": exchange["user"]})
            messages.append({"role": "assistant", "content": exchange["robot"]})

        # موجودہ صارف ان پٹ شامل کریں
        messages.append({"role": "user", "content": user_input})

        return messages

    async def generate_response(
        self,
        context: ConversationContext,
        user_input: str
    ) -> str:
        """
        GPT ماڈل کا استعمال کرکے جواب تیار کریں۔

        Args:
            context: موجودہ مکالمہ سیاق و سباق
            user_input: صارف کا ان پٹ متن

        Returns:
            تیار کردہ جواب متن
        """
        messages = self._build_messages(context, user_input)

        # مظاہرے کے لیے simulate کردہ جواب
        # پیداوار میں، یہ اصل API کو کال کرے گا
        response = await self._call_api(messages)

        return response

    async def _call_api(self, messages: List[Dict[str, str]]) -> str:
        """زبان کے ماڈل کو API کال کریں۔"""
        # اصل API انضمام کے لیے placeholder
        # پیداوار میں openai.AsyncOpenAI کلائنٹ کا استعمال کرے گا
        await asyncio.sleep(0.1)  # API تاخیر simulate کریں

        user_message = messages[-1]["content"] if messages else ""

        # مظاہرے کے لیے سادہ جواب تیار
        response_templates = {
            "greeting": ["Hello! How can I help you today?", "Hi there! What can I do for you?"],
            "command": ["I understand. Let me help with that.", "Got it! I'll take care of that."],
            "question": ["That's a great question. Let me think about that.", "I'd be happy to answer that."],
            "farewell": ["Goodbye! Feel free to ask if you need anything else.", "See you later!"]
        }

        # سادہ ارادے پر مبنی جواب (پیداوار میں اصل NLU کا استعمال کرے گا)
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
    ارادہ درجہ بندی اور ہستی نکالنے کے لیے NLU ماڈیول۔

    رونویشی تقریر سے صارف کے ارادے اور متعلقہ ہستیوں کو نکالتا ہے،
    مناسب مکالمہ انتظام اور روبوٹ کارروائی کے انتخاب کو ممکن بناتا ہے۔
    """

    def __init__(self):
        self.intent_classifier = IntentClassifier()
        self.entity_extractor = EntityExtractor()

    def parse(self, text: str) -> DialogueAct:
        """
        صارف ان پٹ کو ایک منظم مکالمہ فعل میں تشریح کریں۔

        Args:
            text: رونویشی صارف ان پٹ

        Returns:
            ارادے اور ہستیوں کے ساتھ DialogueAct
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
        """تجزیے کے لیے مجموعی اعتماد اسکور کی حساب لگائیں۔"""
        base_confidence = 0.9 if intent != Intent.UNKNOWN else 0.5

        # اگر متعلقہ ارادوں کے لیے ہستیاں ملیں تو اعتماد میں اضافہ کریں
        entity_boost = min(0.1 * len(entities), 0.1)

        return min(base_confidence + entity_boost, 1.0)


class IntentClassifier:
    """
    کلیدی لفظ مماثلت پر مبنی سادہ ارادہ درجہ بندی کنندہ۔

    پیداوار نظاموں میں زیادہ مضبوط ارادہ کی پہچان کے لیے
    تربیت یافتہ درجہ بندی کنندوں یا LLM پر مبنی درجہ بندی کا استعمال کریں گے۔
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
        """صارف ان پٹ کا ارادہ درجہ بندی کریں۔"""
        text_lower = text.lower()

        for intent, patterns in self.INTENT_PATTERNS.items():
            if any(pattern in text_lower for pattern in patterns):
                return intent

        return Intent.UNKNOWN


class EntityExtractor:
    """
    روبوٹ تعاملات میں عام ہستیوں کے لیے ہستی نکالنے والا۔

        قدرتی زبان ان پٹ سے کارروائی کے لیے اشیاء، مقامات، افراد اور مقداروں کی
        شناخا کرتا ہے۔
    """

    ENTITY_TYPES = {
        "object": ["cup", "bottle", "book", "phone", "keys", "bag", "box", "pen"],
        "location": ["kitchen", "living room", "bedroom", "office", "bathroom", "table", "desk"],
        "person": ["me", "you", "him", "her", "them", "my wife", "my husband"],
        "quantity": ["one", "two", "three", "first", "second", "a", "the"]
    }

    def extract(self, text: str) -> List[Entity]:
        """صارف ان پٹ سے ہستیاں نکالیں۔"""
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
    تمام مکالمہ AI اجزاء کو مربوط کرنے والا مکالمہ کا منتظم۔

    مکالمہ حالت مشین کا انتظام کرتا ہے، GPT ماڈل کے ساتھ مربوط ہوتا ہے،
    اور کارروائی کے نفاذ کے لیے روبوٹ کارروائی نظاموں کے ساتھ ہم آہنگ ہوتا ہے۔
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
        """ایک نیا مکالمہ سیشن شروع کریں۔"""
        self.context = ConversationContext(
            session_id=session_id,
            user_id=user_id,
            robot_name=self.robot_name
        )
        self.state = DialogueState.LISTENING

        return f"Hello! I'm {self.robot_name}, your humanoid robot assistant. How can I help you today?"

    async def process_input(self, user_input: str) -> str:
        """
        صارف ان پٹ پر کارروائی کریں اور ایک مناسب جواب تیار کریں۔

        یہ مکالمہ نظام کا اہم داخلی نقطہ ہے، جو ان پٹ سے جواب تک
        مکمل پائپ لائن کو سنبھالتا ہے۔
        """
        if self.context is None:
            raise RuntimeError("Conversation not started. Call start_conversation first.")

        self.state = DialogueState.PROCESSING

        try:
            # مرحلہ 1: قدرتی زبان کی سمجھ
            dialogue_act = self.nlu.parse(user_input)

            # مرحلہ 2: GPT کا استعمال کرکے جواب تیار کریں
            response = await self.gpt_client.generate_response(
                self.context,
                user_input
            )

            # مرحلہ 3: مکالمہ سیاق و سباق کو اپڈیٹ کریں
            self.context.add_exchange(user_input, response)
            self._update_context_from_act(dialogue_act)

            self.state = DialogueState.SPEAKING

            return response

        except Exception as e:
            self.state = DialogueState.ERROR
            return f"I'm sorry, I didn't understand that. Could you please try again?"

    def _update_context_from_act(self, act: DialogueAct) -> None:
        """تجزیے کردہ مکالمہ فعل کے مطابق مکالمہ سیاق و سباق کو اپڈیٹ کریں۔"""
        # ارادے کے مطابق موجودہ موضوع کو اپڈیٹ کریں
        if act.intent == Intent.COMMAND or act.intent == Intent.REQUEST:
            # کام کے سیاق و سباق کے لیے ہستیوں کو نکالیں
            for entity in act.entities:
                if entity.type == "object":
                    self.context.known_entities["target_object"] = entity.value
                elif entity.type == "location":
                    self.context.known_entities["target_location"] = entity.value


# مظاہرے کا استعمال اور توضیح
async def main():
    """مکالمہ نظام کے ساتھ مثالی تعاملات کا مظاہرہ کریں۔"""

    # اجزاء کو شروع کریں
    gpt_client = GPTClient()
    nlu = NaturalLanguageUnderstanding()
    dialogue_manager = DialogueManager(gpt_client, nlu)

    # مکالمہ شروع کریں
    print("=== Fubuni Dialogue System Demo ===\n")

    welcome = await dialogue_manager.start_conversation(
        session_id="demo-session-001",
        user_id="guest"
    )
    print(f"Robot: {welcome}\n")

    # مثالی صارف ان پٹ
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

        # موجودہ حتیکہ چیک کریں
        print(f"[Dialogue State: {dialogue_manager.state.name}]")
        print(f"[Context: {len(dialogue_manager.context.conversation_history)} exchanges]\n")


if __name__ == "__main__":
    asyncio.run(main())

# یہ مثال روبوٹ مکالمہ نظاموں میں GPT انtegration کے اہم فن تعمیر نمط کو ظاہر کرتا ہے۔
# نظام مکالمہ سیاق و سباق برقرار رکھتا ہے، صارف کے ارادے کی تجزیہ کرتا ہے،
# مناسب جوابات تیار کرتا ہے، اور صارف کمانڈز کی بنیاد پر جسمانی کارروائیوں کو نافذ کرنے کے لیے توسیع کیا جا سکتا ہے۔
```

یہ مثال روبوٹ مکالمہ نظاموں میں GPT انضمام کے اہم فن تعمیر نمط کو ظاہر کرتا ہے۔ نظام مکالمہ سیاق و سباق برقرار رکھتا ہے، صارف کے ارادے کی تجزیہ کرتا ہے، مناسب جوابات تیار کرتا ہے، اور صارف کمانڈز کی بنیاد پر جسمانی کارروائیوں کو نافذ کرنے کے لیے توسیع کیا جا سکتا ہے۔

## 6.3 تقریر کی پہچان اور متن سے تقریر

تقریر کی پہچان اور تقریر کی ترکیب مکالمہ روبوٹکس کی آواز انٹرفیس پرت بناتے ہیں۔ یہ ٹیکنالوجیز روبوٹس کو زبانی کمانڈز وصول کرنے اور زبانی تعداد فراہم کرنے کے قابل بناتی ہیں، ایک قدرتی دو طرفہ مواصلاتی چینل پیدا کرتے ہیں۔ ہیومینوئیڈ روبوٹس جو انسانی ماحول میں کام کرتے ہیں، آواز کا تعامل (Voice interaction) رسائی کے فوائد فراہم کرتا ہے اور ہیومینوئیڈ تجسم کے ساتھ آنے والی سماجی توقعات کی حمایت کرتا ہے۔

### خودکار تقریر کی پہچان

جدید خودکار تقریر کی پہچان (ASR) نظام گہرے عصبی نیٹ ورکس کا استعمال کرتے ہیں جو آڈیو لہروں کو متن رونویشیوں میں تبدیل کرتے ہیں۔ اہم فن تعمیر اجزاء میں وہ acoustic ماڈل شامل ہیں جو آڈیو خصوصیات کو فونیمز میں میپ کرتے ہیں، زبان کے ماڈلز جو لفظ کی ترتیبوں کے لیے احتمالی سیاق و سباق فراہم کرتے ہیں، اور تلفظ ماڈلز جو الفاظ کو ان کی فونیٹک نمائندگیوں سے جوڑتے ہیں۔ Whisper جیسے End-to-End ماڈلز نے اس فن تعمیر کو آسان کیا ہے جو واحد عصبی نیٹ ورکس کو تربیت دیتے ہیں جو براہ راست آڈیو کو متن میں میپ کرتے ہیں۔

روبوٹ ایپلیکیشنز کے لیے، تقریر کی پہچان کو کئی عملی چیلنجوں کو سنبھالنا ہوگا۔ HVAC systems، دیگر افراد، یا روبوٹ موٹروں سے پس منظر کی شور درستگی کو خراب کر سکتی ہے۔ کمرے کے پیمانے کی آڈیو کیپچر کے لیے ڈیزائن کی گئی Far-field مائکروفون کنفیگریشنز ایک reverb متعارف کرتی ہے جو acoustic اشارے کو مسخ کرتی ہے۔ متعدد موجودہ اسپیکروں کو source separation یا speaker diarization کی ضرورت ہوتی ہے تاکہ تقریر کو صحیح طور پر منسوب کیا جا سکے۔

مندرجہ ذیل مثال ROS 2 کے لیے ایک تقریر کی پہچان نوڈ نافذ کرتی ہے جو عام ASR خدمات کے ساتھ مربوط ہوتا ہے:

```python
#!/usr/bin/env python3
"""
Speech Recognition Module for Humanoid Robots

روبوٹس کے لیے حقیقی وقت کی تقریر کی پہچان کو ROS 2 کے ساتھ مربوط کرتا ہے،
مقامی اور کلاؤڈ پر مبنی ASR خدمات دونوں کی حمایت کرتا ہے۔
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
    """تقریر کی پراسیسنگ کا نتیجہ۔"""
    transcript: str
    confidence: float
    is_final: bool
    timestamp: float
    audio_energy: float


class AudioPreprocessor:
    """
    تقریر کی پہچان کے لیے آڈیو پری پراسیسنگ پائپ لائن۔

    چیلنجنگ آکوسٹک ماحول میں پہچان کی درستگی بہتر بنانے کے لیے
    آڈیو کی نارملائزیشن، شور کمی، اور خصوصیت نکالنے کو سنبھالتا ہے۔
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
        self.audio_buffer = deque(maxlen=10)  # VAD کے لیے آخری 10 chunks رکھیں
        self.energy_threshold = 500.0  # Voice activity detection threshold
        self.adaptive_threshold = True

    def calculate_energy(self, audio_data: bytes) -> float:
        """آڈیو چنک کا root mean square energy حساب لگائیں۔"""
        try:
            rms = audioop.rms(audio_data, 2)  # 16-bit آڈیو کے لیے 2 bytes per sample
            return float(rms)
        except Exception:
            return 0.0

    def normalize(self, audio_data: bytes) -> bytes:
        """آڈیو کو مستقل volume level پر نارملائز کریں۔"""
        try:
            max_amplitude = max(audioop.max(audio_data, 2), 1)
            scale_factor = 32767.0 / max_amplitude
            normalized = audioop.mul(audio_data, 2, scale_factor)
            return normalized
        except Exception:
            return audio_data

    def apply_noise_reduction(self, audio_data: bytes) -> bytes:
        """
        Spectral subtraction کا استعمال کرتے ہوئے سادہ شور کمی۔

        پیداوار میں، Wiener filtering یا گہری سیکھنے پر مبنی شور دبانے
        جیسے زیادہ پیچیدہ الگورتھم کا استعمال کیا جائے گا۔
        """
        # شور کمی کے لیے placeholder
        # پیداوار میں noisereduce جیسی لائبریریاں استعمال کریں گے
        return audio_data

    def preprocess(self, audio_data: bytes) -> Tuple[bytes, float]:
        """
        پہچان کے لیے آڈیو چنک کو پری پراسیس کریں۔

        Returns:
            (processed_audio, energy_level) کا Tuple
        """
        energy = self.calculate_energy(audio_data)

        # Adaptive threshold ایڈجسٹمنٹ
        if self.adaptive_threshold:
            self.energy_threshold = 0.9 * self.energy_threshold + 0.1 * energy

        # پری پراسیسنگ steps لاگو کریں
        if self.normalize_audio:
            audio_data = self.normalize(audio_data)

        if self.noise_reduction:
            audio_data = self.apply_noise_reduction(audio_data)

        return audio_data, energy

    def is_speech(self, audio_data: bytes) -> bool:
        """Energy threshold کا استعمال کرتے ہوئے voice activity detection۔"""
        energy = self.calculate_energy(audio_data)
        return energy > self.energy_threshold


class ASRProvider(ABC):
    """ASR سروس فراہم کنندگان کے لیے abstract بیس کلاس۔"""

    @abstractmethod
    async def recognize(self, audio_data: bytes) -> SpeechRecognitionResult:
        """آڈیو ڈیٹا پر تقریر کی پہچان کریں۔"""
        pass


class WhisperASR(ASRProvider):
    """
    OpenAI کے Whisper ماڈل کا استعمال کرتے ہوئے Whisper پر مبنی تقریر کی پہچان۔

    مقامی (Whisper.cpp) اور کلاؤڈ (OpenAI API) دونوں deployment کی حمایت کرتا ہے۔
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
        Whisper کا استعمال کرتے ہوئے تقریر کی پہچان کریں۔

        یہ ایک آسان implementation ہے۔ پیداوار کوڈ کرے گا:
        1. آڈیو کو عارضی فائل میں محفوظ کریں (WAV format)
        2. whisper.cpp یا OpenAI API کو subprocess کے ذریعے کال کریں
        3. نتائج کی تجزیہ کریں اور واپس کریں
        """
        import time
        timestamp = time.time()

        # implementation placeholder
        # پیداوار میں، یہ whisper.cpp یا OpenAI API سے رابطہ کرے گا
        await asyncio.sleep(0.05)  # پراسیسنگ وقت simulate کریں

        return SpeechRecognitionResult(
            transcript="",  # اصل رونویشی یہاں ہوگی
            confidence=0.0,
            is_final=False,
            timestamp=timestamp,
            audio_energy=0.0
        )


class ROS2SpeechRecognizer(Node):
    """
    تقریر کی پہچان کے انضمام کے لیے ROS 2 نوڈ۔

    آڈیو topics کو سبسکرائب کرتا ہے، ASR پائپ لائن کے ذریعے آڈیو کو پراسیس کرتا ہے،
    اور پہچان کے نتائج کو شائع کرتا ہے۔
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

        # کنفیگریشن
        self.declare_parameter("confidence_threshold", 0.6)
        self.declare_parameter("partial_results_enabled", True)
        self.declare_parameter("silence_duration", 0.8)  # حتمی بنانے سے پہلے سیکنڈ

        self.confidence_threshold = self.get_parameter("confidence_threshold").value
        self.partial_enabled = self.get_parameter("partial_results_enabled").value

        # حالت
        self.audio_buffer: List[bytes] = []
        self.is_speaking = False
        self.silence_timer: Optional[float] = None

        # آڈیو topics کے لیے QoS پروفائل
        self.audio_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # publishers
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

        # subscriber
        self.audio_sub = self.create_subscription(
            AudioData,
            "audio/raw",
            self.audio_callback,
            self.audio_qos
        )

        self.get_logger().info("Speech Recognizer initialized")

    def audio_callback(self, msg: AudioData) -> None:
        """آنے والی آڈیو ڈیٹا کو پراسیس کریں۔"""
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
                # تقریر کے بعد شور دریافت ہوا - utterence پراسیس کریں
                if self.silence_timer is None:
                    self.silence_timer = self.get_clock().now().nanoseconds / 1e9
                else:
                    silence_duration = (
                        self.get_clock().now().nanoseconds / 1e9 - self.silence_timer
                    )
                    if silence_duration > self.get_parameter("silence_duration").value:
                        self._process_utterance()

    def _process_utterance(self) -> None:
        """جمع کردہ آڈیو buffer کو مکمل utterence کے طور پر پراسیس کریں۔"""
        if not self.audio_buffer:
            return

        # آڈیو chunks کو ملایں
        audio_data = b"".join(self.audio_buffer)
        self.audio_buffer = []
        self.is_speaking = False

        # پہچان کریں
        result = asyncio.run_coroutine_threadsafe(
            self.asr_provider.recognize(audio_data),
            asyncio.new_event_loop()
        ).result()

        # نتائج شائع کریں
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
    روبوٹ تقریر آؤٹ پٹ کے لیے متن سے تقریر ترکیب engine۔

    مختلف TTS فراہم کنندگان کی حمایت کرتا ہے جو مختلف تعامل
        contexts کے لیے آواز کی حسب Madden کے اختیارات رکھتے ہیں۔
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
        متن سے تقریر ترکیب کریں۔

        Args:
            text: ترکیب کے لیے متن
            voice: آواز کی شناخت (language, speaker, etc.)
            speed: تقریر ریٹ multiplier (0.5 to 2.0)
            pitch: Pitch multiplier (0.5 to 2.0)

        Returns:
            WAV format میں آڈیو ڈیٹا
        """
        # implementation placeholder
        # پیداوار میں gTTS، pyttsx3، یا کلاؤڈ TTS services کا استعمال کریں
        await asyncio.sleep(0.1)

        # خالی bytes placeholder واپس کریں
        return b""

    def preprocess_text(self, text: str) -> str:
        """بہتر تقریر ترکیب کے لیے متن کو پری پراسیس کریں۔"""
        # مخففات کو پھیلائیں
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

        # وقفے کے لیے punctuations میں شامل کریں
        processed = processed.replace(".", ". ")
        processed = processed.replace("?", "? ")
        processed = processed.replace("!", "! ")

        return processed.strip()


class ROS2SpeechSynthesizer(Node):
    """
    متن سے تقریر ترکیب کے لیے ROS 2 نوڈ۔

    متن topics کو سبسکرائب کرتا ہے اور ترکیب شدہ آڈیو کو شائع کرتا ہے۔
    """

    def __init__(self, tts_engine: TextToSpeechEngine):
        super().__init__("speech_synthesizer")

        self.tts_engine = tts_engine

        # QoS پروفائل
        self.text_qos = QoSProfile(depth=10)

        # آڈیو کے لیے publisher
        self.audio_pub = self.create_publisher(
            AudioData,
            "speech/audio",
            10
        )

        # متن ان پٹ کے لیے subscriber
        self.text_sub = self.create_subscription(
            String,
            "speech/text",
            self.synthesize_callback,
            self.text_qos
        )

        self.get_logger().info("Speech Synthesizer initialized")

    def synthesize_callback(self, msg: String) -> None:
        """متن کو پراسیس کریں اور تقریر ترکیب کریں۔"""
        text = msg.data

        # متن کو پری پراسیس کریں
        text = self.tts_engine.preprocess_text(text)

        # ترکیب کریں
        audio_data = asyncio.run(
            self.tts_engine.synthesize(text)
        )

        if audio_data:
            # آڈیو شائع کریں
            audio_msg = AudioData()
            audio_msg.data = audio_data
            self.audio_pub.publish(audio_msg)

            self.get_logger().info(f"Synthesized speech for: '{text[:50]}...'")


# تقریر کی پہچان پائپ لائن کا مظاہرہ
async def demonstrate_speech_pipeline():
    """تقریر کی پہچان اور ترکیب پائپ لائن کا مظاہرہ کریں۔"""

    print("=== Speech Recognition Pipeline Demo ===\n")

    # اجزاء کو شروع کریں
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

    # TTS مظاہرہ
    tts = TextToSpeechEngine(default_voice="en-US-Female")
    print("Text-to-Speech engine initialized:")
    print(f"  - Default voice: {tts.default_voice}\n")

    print("Pipeline components ready for ROS 2 integration.")
    print("Would subscribe to audio topics and publish recognized text.")


if __name__ == "__main__":
    asyncio.run(demonstrate_speech_pipeline())

# یہ مکیول روبوٹس کے لیے آواز کے انٹرفیس کے اہم اجزاء کو ظاہر کرتا ہے۔
# AudioPreprocessor کلاس آڈیو کی نارملائزیشن، شور کمی، اور voice activity detection کو سنبھالتی ہے۔
# ROS2SpeechRecognizer نوڈ آڈیو topics کو پراسیس کرتا ہے اور پہچان کے نتائج کو شائع کرتا ہے۔
# TextToSpeechEngine اور ROS2SpeechSynthesizer متن سے تقریر ترکیب کو نافذ کرتے ہیں۔
```

## 6.4 قدرتی زبان کی سمجھ

قدرتی زبان کی سمجھ (Natural language understanding) خام رونویشی ٹیکسٹ کو منظم نمائندگیوں میں تبدیل کرتی ہے جو روبوٹ نظام عمل کر سکتے ہیں۔ اہم کاموں میں ارادہ درجہ بندی، ہستی نکالنا، حوالہ حل، اور مکالمہ فعل کی تشریح شامل ہے۔ روبوٹ ایپلیکیشنز کے لیے، NLU کو قدرتی انسانی مواصلات کی خصوصیت والی غیر درست اور نامکمل زبان کے لیے مضبوط ہونا چاہیے۔

ارادہ درجہ بندی صارف کے بیانات کو پہلے سے متعینہ زمرے میں تفویض کرتی ہے جو روبوٹ کارروائیوں یا مکالمہ رویوں کو میپ کرتے ہیں۔ ہیومینوئڈ روبوٹس کے لیے عام ارادے میں سلام، کمانڈ، سوالات، معلومات کی درخواستیں، اور وضاحت طلب کرنا یا سمجھنے کی تصدیق کرنا جیسے مکالمہ انتظام افعال شامل ہیں۔ ملٹی لیبل درجہ بندی بیانات کو ایک ساتھ متعدد ارادے رکھنے کی اجازت دیتی ہے، جیسے کہ درخواست کے ساتھ ملایا گیا سلام۔

ہستی نکالنا صارف کے بیانات میں مخصوص معلومات کی ٹکڑوں کی شناخت کرتی ہے۔ روبوٹ ایپلیکیشنز کے لیے، اہم ہستی کی اقسام میں وہ اشیاء شامل ہیں جنہیں روبوٹ ہینڈل کر سکتا ہے، ماحول میں مقامات، روبوٹ سے تعامل کرنے والے افراد، اور وہ وقتی اظہار جو بتاتے ہیں کہ کب کارروائی ہونی چاہیے۔ ہستی نکالنا صريح ذکر اور پہلے سے زیر بحث ہستیوں کے حوالوں دونوں کو سنبھالنے چاہیے۔

حوالہ حل ضمائر، اشارہ دینے والے، اور دیگر anaphoric اظہار کو ان کے حوالہ ہستیوں سے جوڑتا ہے۔ جب صارف کہتا ہے "مجھے کپ دے اور اسے میز پر رکھ دو"، ضمیر "یہ" کپ کا حوالہ دیتا ہے، میز کا نہیں۔ حوالہ حل کے لیے مکالمہ ادوار میں ڈسکورس ہستیوں کو ٹریک کرنا اور جسمانی سیاق و سباق کو سمجھنا ضروری ہے جس میں حوالے ہوتے ہیں۔

مندرجہ ذیل مثال ارادہ درجہ بندی، ہستی نکالنا، اور حوالہ حل کے ساتھ ایک جامع NLU پائپ لائن نافذ کرتی ہے:

```python
#!/usr/bin/env python3
"""
Natural Language Understanding for Humanoid Robots

ہیومینوئڈ روبوٹس کے لیے جامع NLU اہلیتیں فراہم کرتا ہے جس میں
ارادہ درجہ بندی، ہستی نکالنا، حوالہ حل، اور семантиک تجزیہ شامل ہے۔
"""

import re
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from typing import Any, Dict, List, Optional, Set, Tuple


class Intent(Enum):
    """روبوٹ تعاملات کے جامع ارادے۔"""
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
    """ہستی کو خصوصیت، قدر، اور metadata کے ساتھ نکالا گیا۔"""
    type: str
    value: str
    raw_text: str
    position: Tuple[int, int]
    confidence: float = 1.0
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class Reference:
    """پہلے سے زیر بحث ہستی کا anaphoric حوالہ۔"""
    text: str
    position: Tuple[int, int]
    referent_type: str
    antecedent: Optional[str] = None
    confidence: float = 1.0


@dataclass
class NLUParsing:
    """NLU تجزیے کا مکمل نتیجہ۔"""
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
    روبوٹ تعامل contexts میں متعدد قسم کا ہستی نکالنے والا۔

    قدرتی زبان ان پٹ سے نمونہ مماثلت اور درجہ بندی کا استعمال کرتے ہوئے
    اشیاء، مقامات، افراد، مقداریں، اور کارروائیاں نکالتا ہے۔
    """

    # ہستی کے نمونے
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
        """fuzzy مماثلت کے لیے جانی گئی ہستی قدر کی set بنائیں۔"""
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
        """ٹیکسٹ سے تمام ہستیاں نکالیں۔"""
        entities = []
        text_lower = text.lower()

        for entity_type, patterns in self.ENTITY_PATTERNS.items():
            for pattern in patterns:
                for match in re.finditer(pattern, text_lower):
                    value = match.group().strip()

                    # quantity والے اظہار کو صاف کریں
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

        # پوزیشن کے لحاظ سے ترتیب دیں
        entities.sort(key=lambda e: e.position[0])

        return entities

    def _normalize_entity_value(self, entity_type: str, value: str) -> str:
        """نکالی گئی ہستی قدر کو نارملائز کریں۔"""
        # articles اور quantifiers ہٹائیں
        stopwords = {"a", "an", "the", "some", "few", "several", "many"}
        words = value.split()
        filtered = [w for w in words if w not in stopwords]
        return " ".join(filtered) if filtered else value

    def _calculate_confidence(self, entity_type: str, value: str) -> float:
        """نمونے کی مماثلت کے معیار کی بنیاد پر نکالنے کا اعتماد۔"""
        # براہ راست ڈکشنری مماثلتوں کا اعتماد زیادہ ہے
        if value.lower() in self.entity_values.get(entity_type, set()):
            return 0.95
        return 0.75


class IntentClassifier:
    """
    نمونے کی مماثلت اور семантиک خصوصیات کا استعمال کرتے ہوئے ارادہ درجہ بندی کنندہ۔

    پیداوار نظاموں میں تربیت یافتہ classifiers کا استعمال کریں گے، لیکن نمونے پر مبنی
    درجہ بندی قابل تفسیر اور قابل ترمیم قواعد فراہم کرتی ہے۔
    """

    # متعلقہ اعتماد اوزان کے ساتھ ارادے کے نمونے
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
        صارف ٹیکسٹ کا ارادہ درجہ بندی کریں۔

        Returns:
            (intent, confidence_score) کا Tuple
        """
        text_lower = text.lower()
        best_intent = Intent.UNKNOWN
        best_confidence = 0.0

        for intent, patterns in self.INTENT_PATTERNS.items():
            for pattern, base_confidence in patterns:
                if re.search(pattern, text_lower):
                    # مماثلت کی تفصیلات کے مطابق اعتماد ایڈجسٹ کریں
                    confidence = base_confidence

                    # اگر نمونہ پورے ٹیکسٹ سے مماثل ہو
                    if re.fullmatch(pattern, text_lower):
                        confidence = min(confidence * 1.1, 1.0)

                    if confidence > best_confidence:
                        best_confidence = confidence
                        best_intent = intent

        return best_intent, best_confidence


class ReferenceResolver:
    """
    ضمائر اور تفصیلات کے لیے anaphoric حوالہ حل۔

    ڈسکورس ہستیوں کو ٹریک کرتا ہے اور مکالمہ ادوار میں
    حوالوں کو حل کرتا ہے۔
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
        موجودہ utterance میں حوالوں کو حل کریں۔

        Args:
            text: موجودہ utterance ٹیکسٹ
            entities: موجودہ utterance سے نکالی گئی ہستیاں

        Returns:
            حل شدہ حوالوں کی فہرست
        """
        references = []

        # ضمائر کو حل کریں
        for match in re.finditer(r"\b(it|they|them|he|she|him|her|this|that)\b", text.lower()):
            reference_text = match.group()
            pos = (match.start(), match.end())

            # ممکنہ referent types حاصل کریں
            referent_types = self.PRONOUN_PATTERNS.get(reference_text, ["object"])

            # بہترین antecedent تلاش کریں
            antecedent = self._find_antecedent(referent_types, entities)

            if antecedent:
                references.append(Reference(
                    text=reference_text,
                    position=pos,
                    referent_type=referent_types[0],
                    antecedent=antecedent,
                    confidence=0.85
                ))

        # ذکر کی تاریخ کو اپڈیٹ کریں
        for entity in entities:
            self.mention_history.append({
                "type": entity.type,
                "value": entity.value,
                "text": entity.raw_text,
                "timestamp": datetime.now().isoformat()
            })

        # حالیہ ذکر ہی رکھیں
        max_history = 10
        if len(self.mention_history) > max_history:
            self.mention_history = self.mention_history[-max_history:]

        return references

    def _find_antecedent(
        self,
        referent_types: List[str],
        current_entities: List[Entity]
    ) -> Optional[str]:
        """حوالے کے لیے سبقت رکھنے والے کو تلاش کریں۔"""
        # فوری حوالے کے لیے موجودہ utterance میں پہلے چیک کریں
        for entity in current_entities:
            if entity.type in referent_types:
                return entity.value

        # پھر حالیہ تاریخ چیک کریں
        for mention in reversed(self.mention_history):
            if mention["type"] in referent_types:
                return mention["value"]

        return None


class NaturalLanguageUnderstanding:
    """
    تمام اجزاء کو ملا کر مکمل NLU پائپ لائن۔

    ارادہ درجہ بندی، ہستی نکالنا، اور حوالہ حل کو
    یکجا تجزیہ انٹرفیس میں مربوط کرتا ہے۔
    """

    def __init__(self):
        self.intent_classifier = IntentClassifier()
        self.entity_extractor = EntityExtractor()
        self.reference_resolver = ReferenceResolver()

    def parse(self, text: str) -> NLUParsing:
        """
        صارف ان پٹ کا مکمل NLU تجزیہ۔

        Args:
            text: خام صارف ان پٹ ٹیکسٹ

        Returns:
            تمام تجزیہ شدہ اجزاء کے ساتھ مکمل NLUParsing
        """
        # ہستیاں نکالیں
        entities = self.entity_extractor.extract(text)

        # ارادہ درجہ بندی کریں
        intent, intent_confidence = self.intent_classifier.classify(text)

        # حوالے حل کریں
        references = self.reference_resolver.resolve(text, entities)

        # اضافی خصوصیات کا حساب لگائیں
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
        """ان پٹ کی جذباتی polarity کا تجزیہ (-1 سے 1)۔"""
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
        """ان پٹ کی urgency کا تجزیہ (0 سے 1)۔"""
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
        urgency = 0.1  # بنیادی urgency

        for indicator, level in urgency_indicators.items():
            if indicator in text_lower:
                urgency = max(urgency, level)

        # سوالات عام طور پر کم urgent ہوتے ہیں
        if text.strip().endswith("?"):
            urgency *= 0.7

        return urgency

    def _analyze_politeness(self, text: str) -> float:
        """ان پٹ میں شائستگی markers کا تجزیہ (0 سے 1)۔"""
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
        politeness = 0.5  # بنیادی شائستگی

        for marker, level in polite_markers.items():
            if marker in text_lower:
                politeness = max(politeness, level)

        return politeness


# NLU پائپ لائن کا مظاہرہ
def demonstrate_nlu():
    """مثالی ان پٹ کے ساتھ NLU پائپ لائن کا مظاہرہ کریں۔"""

    print("=== Natural Language Understanding Demo ===\n")

    # NLU شروع کریں
    nlu = NaturalLanguageUnderstanding()

    # مثالی ان پٹ
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

# یہ مثال ہیومینوئڈ روبوٹس کے لیے ایک جامع NLU پائپ لائن نافذ کرتی ہے۔
# EntityExtractor کلاس اشیاء، مقامات، افراد، مقداریں، اور کارروائیاں نکالتی ہے۔
# IntentClassifier کلاس صارف کے ارادے کو درجہ بندی کرتی ہے۔
# ReferenceResolver کلاس ضمائر اور اشارہ دینے والے اظہار کو حل کرتی ہے۔
# NaturalLanguageUnderstanding کلاس سبھی اجزاء کو مربوط کرتی ہے۔
```

## 6.5 کثیرالطریقہ تعامل

ہیومینوئیڈ روبوٹ تعامل قدرتی طور پر تقریر سے آگے بڑھ کر اشارے، چہرے کا ظہور، آنکھ کی نظر، اور جسمانی کارروائی کو شامل کرتا ہے۔ کثیرالطریقہ تعامل نظام ان چینلوں کو ملا کر زیادہ قدرتی اور جذباتی مواصلات بناتے ہیں۔ ہیومینوئیڈ روبوٹس کے لیے، لفظی اور غیر لفظی اشارات کی صف بندی سالم سماجی تعامل کے لیے ضروری ہے۔

اشارہ کی پہچان روبوٹس کو اشارہ کرنے، بلانے، ہلानے، اور دیگر مواصلاتی اشاروں کو سمجھنے اور ان پر رد عمل دینے کے قابل بناتی ہے۔ کمپیوٹر وژن سسٹم ہاتھ کی پوزیشنوں کو ٹریک کرتے ہیں اور پہلے سے متعینہ اشارے کے vocabulary کو پہچانتے ہیں۔ تقریر کے ساتھ مل کر، اشارے ایسی الجھن کو حل کرتے ہیں جو بولے گئے کمانڈز میں مبہم ہوتی ہے۔ ایک صارف جو اشیاء کی طرف اشارہ کرتے ہوئے کہے "مجھے یہ دو" صاف حوالہ فراہم کرتا ہے جہاں تقریر اکیلی مبہم ہوگی۔

چہرے کے ظہور کی پہچان اور پیداوار انسان روبوٹ تعامل میں جذباتی تھریبک فیڈback loops بناتی ہے۔ جو صارفین چہرے کے اظہار سے جذبات کو پہچان سکتے ں وہ اپنے جوابات کو مماثل طور پر ایڈجسٹ کر سکتے ہیں۔ اسی طرح، جو روبوٹس مناسب چہرے کے اظہار دکھاتے ہیں وہ اپنی داخلی حالت کو مواصلات کرتے ہیں اور زیادہ مشغول تعامل بناتے ہیں۔ جذباتی آنکھیں، ابرو، اور منہ کی حرکات تقریر کو جذباتی مواد سے supplement کرتی ہیں۔

آنکھ کی نظر کی سمت سماجی تعامل میں توجہ اور ارادے کو مواصلات کرتی ہے۔ انسان قدرتی طور پر اپنے مکالمے کے ساتھیوں کی نظر کی پیروی کرتے ہیں تاکہ سمجھ سکیں کہ وہ کس چیز پر توجہ دے رہے ہیں۔ ہیومینوئیڈ روبوٹس صارف کی توجہ کو سمجھنے کے لیے آنکھ ٹریکنگ کا استعمال کر سکتے ہیں اور مشترکہ توجہ کے رویوں کی حمایت کے لیے اپنی نظر خود ہدایت کر سکتے ہیں۔ سوچنے یا سننے کے دوران نظر چرائیں انسانی سماجی نمط کی نقالی کرتی ہے اور روبوٹ کے رویے کو زیادہ فطری محسوس کراتی ہے۔

مندرجہ ذیل مثال ایک کثیرالطریقہ انضمام پرت نافذ کرتی ہے جو تقریر، اشارے، اور چہرے کے ظہور کو ہم آہنگ کرتی ہے:

```python
#!/usr/bin/env python3
"""
Multi-Modal Interaction System for Humanoid Robots

قدرتی انسان روبوٹ مواصلات کے لیے تقریر، اشارے، چہرے کا ظہور، اور نظر کو
کثیرالطریقہ چینلز میں مربوط کرتا ہے۔
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple
from abc import ABC, abstractmethod
import asyncio


class Modality(Enum):
    """تعامل کے طریقوں کی فہرست۔"""
    SPEECH = "speech"
    GESTURE = "gesture"
    FACIAL_EXPRESSION = "facial"
    EYE_GAZE = "gaze"
    BODY_POSTURE = "posture"
    TOUCH = "touch"


class GestureType(Enum):
    """روبوٹ execution کے لیے پہلے سے متعینہ اشارے کی اقسام۔"""
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
    """روبوٹ ڈسپلے کے لیے چہرے کے اظہارات۔"""
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
    """آنکھ کی حرکت کے لیے نظر کے ہدف۔"""
    SPEAKER = "speaker"
    OBJECT = "object"
    LOCATION = "location"
    DOWN = "down"  # مطیع/احترام آمیز
    UP = "up"      # سوچنے/غور کرنے والا
    AWAY = "away"  # نظر چرانے
    NEUTRAL = "neutral"


@dataclass
class MultiModalMessage:
    """
    چینلز میں ہم وقت ظہور کے ساتھ کثیرالطریقہ پیغام۔

    تقریر، اشارے، چہرے کا ظہور، اور نظر کو ایک
    متفقہ مواصلاتی پیکیج میں ہم آہنگ کرتا ہے۔
    """
    text: str
    gesture: Optional[GestureType] = None
    facial_expression: FacialExpression = FacialExpression.NEUTRAL
    gaze_target: GazeTarget = GazeTarget.SPEAKER
    emotion_intensity: float = 0.5  # 0.0 سے 1.0 تک
    duration: float = 2.0  # اشارے/ظہور ڈسپلے کے لیے سیکنڈ
    priority: int = 0  # زیادہ priority کم کو interrupt کرتا ہے
    timestamp: float = field(default_factory=lambda: __import__('time').time())


class GestureController:
    """
    روبوٹ اشارہ execution کے لیے کنٹرولر۔

    اشارے کی ترتیب، وقت کو سنبھالتا ہے اور دیگر
    تعامل کے طریقوں کے ساتھ ہم آہنگ ہوتا ہے۔
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
        مخصوص duration اور تکرار کے ساتھ ایک اشارہ نافذ کریں۔

        Args:
            gesture: نافذ کرنے کے لیے اشارہ
            duration: ہر اشارے کے چکر کی مدت سیکنڈوں میں
            repeat: اشارہ دہرانے کی تعداد

        Returns:
            اگر اشارہ کامیابی سے نافذ ہوا
        """
        self.current_gesture = gesture
        self.is_executing = True

        for i in range(repeat):
            print(f"[Gesture] Executing: {gesture.value}")
            await asyncio.sleep(duration)

            # interruption کے لیے چیک کریں
            if not self.is_executing:
                return False

        self.is_executing = False
        self.current_gesture = None
        return True

    def queue_gesture(self, message: MultiModalMessage) -> None:
        """اشارہ execution کے لیے کثیرالطریقہ پیغام queue کریں۔"""
        self.gesture_queue.append(message)

    async def process_queue(self) -> None:
        """ترتیب priority کے مطابق queued اشارے پراسیس کریں۔"""
        while self.gesture_queue:
            # priority کے مطابق ترتیب دیں
            self.gesture_queue.sort(key=lambda m: m.priority, reverse=True)

            message = self.gesture_queue.pop(0)

            if message.gesture:
                await self.execute_gesture(
                    message.gesture,
                    duration=message.duration
                )


class FacialExpressionController:
    """
    روبوٹ چہرے کے ڈسپلے کے لیے کنٹرولر۔

    اظہار کی تبدیلیوں، blends، اور وقت کو
        جذباتی طور پر expressive روبوٹ چہروں کے لیے سنبھالتا ہے۔
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
        موجودہ چہرے کا اظہار مقرر کریں۔

        Args:
            expression: ڈسپلے کرنے کے لیے اظہار
            intensity: اظہار کی شدت (0.0 سے 1.0)
            duration: نیوٹرل واپس آنے سے پہلے اختیاری مدت
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
        """مدت ختم ہونے کے بعد نیوٹرل پر ری سیٹ کریں۔"""
        await asyncio.sleep(duration)
        self.set_expression(FacialExpression.NEUTRAL, 0.0)

    def get_emotion_for_expression(
        self,
        expression: FacialExpression
    ) -> Dict[str, float]:
        """چہرے کے اظہار سے مطابقت رکھنے والی جذباتی سکور حاصل کریں۔"""
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
    روبوٹ آنکھ کی نظر کی سمت کے لیے کنٹرولر۔

    قدرتی سماجی نظر کے رویوں کے لیے آنکھ کی حرکات کا انتظام کرتا ہے
    بشمول مشترکہ توجہ اور مکالمہ نظر کے نمط۔
    """

    def __init__(self):
        self.current_target: GazeTarget = GazeTarget.NEUTRAL
        self.gaze_position: Tuple[float, float] = (0.0, 0.0)  # x, y زاویے
        self.is_tracking = False
        self.tracking_target: Optional[str] = None

    def set_gaze(
        self,
        target: GazeTarget,
        position: Optional[Tuple[float, float]] = None,
        smooth: bool = True
    ) -> None:
        """
        نظر کی سمت مقرر کریں۔

        Args:
            target: نظر کے ہدف کی قسم
            position: object/location gazes کے لیے مخصوص پوزیشن
            smooth: کیا ہموار آنکھ کی حرکت استعمال کریں
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
        """نظر کے ہدف کو آنکھ کے زاویوں میں تبدیل کریں۔"""
        angle_map = {
            GazeTarget.SPEAKER: (0.0, 5.0),    # وسط، تھوڑا اوپر
            GazeTarget.NEUTRAL: (0.0, 0.0),    # وسط
            GazeTarget.DOWN: (0.0, -15.0),     # نیچے دیکھنا
            GazeTarget.UP: (0.0, 15.0),        # اوپر دیکھنا
            GazeTarget.AWAY: (30.0, 0.0),      # دور دیکھنا
        }

        if target == GazeTarget.OBJECT or target == GazeTarget.LOCATION:
            return position if position else (0.0, 0.0)

        return angle_map.get(target, (0.0, 0.0))

    async def gaze_at_person(self, person_position: Tuple[float, float]) -> None:
        """نظر کے ساتھ ایک خاص شخص کی پوزیشن ٹریک کریں۔"""
        self.is_tracking = True
        self.tracking_target = "person"

        # اپڈیٹس کے ساتھ ٹریکنگ simulate کریں
        while self.is_tracking:
            self.gaze_position = person_position
            await asyncio.sleep(0.1)

    def stop_tracking(self) -> None:
        """ٹریکنگ روکیں اور نیوٹرل پر واپس آئیں۔"""
        self.is_tracking = False
        self.tracking_target = None
        self.set_gaze(GazeTarget.NEUTRAL)


class MultiModalIntegrator:
    """
    کثیرالطریقہ روبوٹ تعامل کے لیے اہم integrator۔

    تقریر، اشارے، چہرے کا ظہور، اور نظر کو
    متفقہ، سیاق و سباق کے مطابق جوابات میں ہم آہنگ کرتا ہے۔
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
        """ایک مخصوص طریقے کو فعال یا غیر فعال کریں۔"""
        self.modality_enabled[modality] = enabled

    async def express(
        self,
        message: MultiModalMessage,
        async_speech: bool = True
    ) -> None:
        """
        تمام فعال چینلز پر ایک کثیرالطریقہ پیغام ظاہر کریں۔

        اشارے، چہرے، اور نظر کے طریقوں میں اظہار کے وقت اور نفاذ کو ہم آہنگ کرتا ہے۔
        """
        tasks = []

        # اشارہ
        if message.gesture and self.modality_enabled[Modality.GESTURE]:
            tasks.append(self.gesture_controller.execute_gesture(
                message.gesture,
                duration=message.duration
            ))

        # چہرے کا اظہار
        if self.modality_enabled[Modality.FACIAL_EXPRESSION]:
            self.face_controller.set_expression(
                message.facial_expression,
                intensity=message.emotion_intensity,
                duration=message.duration
            )

        # نظر
        if self.modality_enabled[Modality.EYE_GAZE]:
            self.gaze_controller.set_gaze(message.gaze_target)

        # سبھی کو concurrently نافذ کریں
        if tasks:
            await asyncio.gather(*tasks)

        # مدت کے بعد نیوٹرل پر واپس آئیں
        await asyncio.sleep(message.duration * 0.5)
        self.face_controller.set_expression(FacialExpression.NEUTRAL, 0.0)
        self.gaze_controller.set_gaze(GazeTarget.NEUTRAL)

    def create_happy_response(self, text: str) -> MultiModalMessage:
        """ایک خوش کثیرالطریقہ جواب بنائیں۔"""
        return MultiModalMessage(
            text=text,
            facial_expression=FacialExpression.SMILING,
            gaze_target=GazeTarget.SPEAKER,
            emotion_intensity=0.7,
            gesture=GestureType.NOD
        )

    def create_thinking_response(self, text: str) -> MultiModalMessage:
        """ایک غور کرنے والا کثیرالطریقہ جواب بنائیں۔"""
        return MultiModalMessage(
            text=text,
            facial_expression=FacialExpression.THINKING,
            gaze_target=GazeTarget.UP,
            emotion_intensity=0.4
        )

    def create_confused_response(self, text: str) -> MultiModalMessage:
        """وضاحت کے لیے ایک پریشان کثیرالطریقہ جواب بنائیں۔"""
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
        """اشارہ کے ساتھ ایک جواب بنائیں۔"""
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


# کثیرالطریقہ تعامل کا مظاہرہ
async def demonstrate_multi_modal():
    """کثیرالطریقہ تعامل نظام کا مظاہرہ کریں۔"""

    print("=== Multi-Modal Interaction Demo ===\n")

    # integrator شروع کریں
    integrator = MultiModalIntegrator()

    # مثالی کثیرالطریقہ پیغامات
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

# یہ مثال ہیومینوئیڈ روبوٹس کے لیے ایک مکمل کثیرالطریقہ انضمام نظام نافذ کرتی ہے۔
# GestureController کلاس روبوٹ اشاروں کے execution کو سنبھالتی ہے۔
# FacialExpressionController کلاس چہرے کے اظہارات کو کنٹرول کرتی ہے۔
# GazeController کلاس آنکھ کی نظر کی سمت کا انتظام کرتی ہے۔
# MultiModalIntegrator کلاس سبھی طریقوں کو ہم آہنگ کرتی ہے۔
```

## باب کا خلاصہ

اس باب نے ہیومینوئیڈ روبوٹس کے لیے مکالمہ AI کے جامع موضوع کی کھوج کی، قدرتی انسان روبوٹ مکالمے کو ممکن بنانے والی اہم ٹیکنالوجیز اور انضمام کے نمط کا احاطہ کیا۔

1. **مکالمہ AI کی بنیادیں**: اس باب نے روبوٹکس کے لیے مکالمہ AI پائپ لائن متعارف کی، وضاحت کرتے ہوئے کہ تقریر کی پہچان، قدرتی زبان کی سمجھ، مکالمہ کا انتظام، اور تقریر کی ترکیب کس طرح مل کر قدرتی تعامل کے تجربے بناتے ہیں۔

2. **GPT انضمام**: ہم نے روبوٹ مکالمہ نظاموں میں GPT جیسے بڑے زبان کے ماڈلز کو مربوط کرنے کے نمطوں کی جانچ کی، بشمول سیاق و سباق کا انتظام، پرامپٹ انجینئرنگ، اور کلاؤڈ اور مقامی پراسیسنگ کو ملا کر ہائبرڈ فن تعمیر۔

3. **تقریر کی پہچان اور ترکیب**: اس باب نے آواز کے انٹرفیس کے عملی نفاذ کا احاطہ کیا بشمول آڈیو پری پراسیسنگ، voice activity detection، اور روبوٹ آؤٹ پٹ کے لیے متن سے تقریر ترکیب۔

4. **قدرتی زبان کی سمجھ**: ہم نے جامع NLU اہلیتیں نافذ کیں بشمول ارادہ درجہ بندی، ہستی نکالنا، حوالہ حل، اور صارف ان پٹ کو سمجھنے کے لیے جذباتی تجزیہ۔

5. **کثیرالطریقہ تعامل**: اس باب نے تلاش کیا کہ تقریر اشاروں، چہرے کے ظہور، اور آنکھ کی نظر کے ساتھ کس طرح مربوط ہوتی ہے تاکہ کثیرالطریقہ چینلز میں متفقہ، جذباتی روبوٹ مواصلات بن سکیں۔

### اہم تصورات

- **مکالمہ پائپ لائن**: وائس تعامل کو ممکن بنانے والی ASR، NLU، مکالمہ کا انتظام، NLG، اور TTS کی ترتیب
- **ارادہ درجہ بندی**: مناسب جواب کے انتخاب کے لیے صارف بیانات کو پہلے سے متعینہ زمرے میں تفویض کرنا
- **ہستی نکالنا**: صارف تقریر میں مخصوص اشیاء، مقامات اور دیگر متعلقہ معلومات کی شناخت
- **کثیرالطریقہ انضمام**: قدرتی اظہار کے لیے لفظی اور غیر لفظی مواصلاتی چینلوں کی ہم آہنگی
- **سیاق و سباق کا انتظار**: توسیع تعاملات میں مکالمہ حالت برقرار رکھنا

### اہم اصطلاحات

- **ASR (Automatic Speech Recognition)**: تقریر کو متن میں تبدیل کرنے کی ٹیکنالوجی
- **TTS (Text-to-Speech)**: متن کو بولی ہوئی زبان میں تبدیل کرنے کی ٹیکنالوجی
- **NLU (Natural Language Understanding)**: معنی اور ارادہ نکالنے کے لیے زبان کی پراسیسنگ
- **مکالمہ کا منتظم**: مکالمہ حالت برقرار رکھنے اور جوابات کو ہم آہنگ کرنے والا نظام کا جزو
- **کثیرالطریقہ تعامل**: متعدد مواصلاتی چینلز کا ایک ساتھ استعمال

### مزید پڑھنا

- **Speech and Language Processing**: Jurafsky & Martin - جامع NLP درسی کتاب
- **OpenAI Whisper Documentation**: جدید ترین تقریر کی پہچان
- **ROS 2 Audio Packages**: ROS کے لیے آڈیو پراسیسنگ اور ASR انضمام
- **Human-Robot Interaction Research**: سماجی روبوٹکس پمعاصری پرچر

### اگلا باب

باب 7 **Robot Learning and Adaptation** کی کھوج کرتا ہے، جو بتاتا ہے کہ روبوٹس تجربے سے کیسے سیکھتے ہیں، نئی صورت حالوں کو کیسے اپناتے ہیں، اور مشین سیکھنے کی تکنیکوں کے ذریعے وقت کے ساتھ اپنی اہلیتوں کو کیسے بہتر بناتے ہیں۔

---

**حصہ 6: مکالمہ روبوٹکس** | [باب 6: مکالمہ روبوٹکس](part-6-conversational/conversational-robotics) | [ضمیمہ: تشخیص روبکس](appendix/D-assessment-rubrics)
