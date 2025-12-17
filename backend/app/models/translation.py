from sqlmodel import SQLModel, Field
from datetime import datetime
from pydantic import BaseModel
from typing import Optional

class ChapterTranslationDB(SQLModel, table=True):
    __tablename__ = "chapter_translations"

    id: Optional[int] = Field(default=None, primary_key=True)
    chapter_id: str = Field(max_length=255, nullable=False)
    locale: str = Field(max_length=10, default='ur', nullable=False)
    content: str = Field(nullable=False)
    translated_by: Optional[int] = Field(default=None)  # Keep as int reference but no foreign key
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)

    class Config:
        from_attributes = True


# Pydantic models for API
class TranslationBase(BaseModel):
    chapter_id: str
    locale: str
    content: str


class TranslationCreate(TranslationBase):
    pass


class TranslationUpdate(TranslationBase):
    pass


class Translation(TranslationBase):
    id: int
    translated_by: Optional[int] = None
    created_at: str
    updated_at: str

    class Config:
        from_attributes = True


class TranslationResponse(BaseModel):
    exists: bool
    translation: Optional[Translation] = None


class TranslationRequest(BaseModel):
    chapterId: str
    content: str
    targetLocale: str


class SaveTranslationRequest(BaseModel):
    chapterId: str
    locale: str
    content: str


class GetTranslationResponse(BaseModel):
    exists: bool
    translation: Optional[Translation] = None