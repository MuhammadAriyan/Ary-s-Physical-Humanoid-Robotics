from pydantic_settings import BaseSettings
from typing import List, Union
import os


class Settings(BaseSettings):
    database_url: str = os.getenv("NEON_DATABASE_URL", "sqlite:///./test.db")
    openrouter_api_key: str = os.getenv("OPENROUTER_API_KEY", "")
    openai_base_url: str = os.getenv("OPENAI_BASE_URL", "https://openrouter.ai/api/v1")
    app_env: str = os.getenv("APP_ENV", "development")
    backend_cors_origins: Union[str, List[str]] = os.getenv(
        "BACKEND_CORS_ORIGINS", '["http://localhost:3000", "http://localhost:3001"]'
    )

    # Qdrant settings
    qdrant_url: str = os.getenv("QDRANT_URL", "http://localhost:6333")
    qdrant_api_key: str = os.getenv("QDRANT_API_KEY", "")
    qdrant_collection_name: str = os.getenv("QDRANT_COLLECTION_NAME", "robotics_docs")

    model_config = {"extra": "ignore"}


settings = Settings()
