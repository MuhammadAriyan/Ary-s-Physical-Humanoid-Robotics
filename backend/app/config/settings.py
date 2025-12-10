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

    model_config = {"extra": "ignore"}


settings = Settings()
