from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from .config.settings import settings
from .api import chat, test_chat, rag
from .database.init import create_db_and_tables
import json


def create_app():
    app = FastAPI(title="Fubuni Chat API", version="1.0.0")

    # Configure CORS
    origins = []
    if settings.app_env == "development":
        origins = ["*"]  # Allow all in development
    else:
        # Parse the cors origins from settings (could be a string or list)
        if isinstance(settings.backend_cors_origins, str):
            origins = json.loads(settings.backend_cors_origins)
        else:
            origins = settings.backend_cors_origins

    app.add_middleware(
        CORSMiddleware,
        allow_origins=origins,
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    # Include API routes
    app.include_router(chat.router, prefix="/api", tags=["chat"])
    app.include_router(test_chat.router, prefix="/api", tags=["test"])
    app.include_router(rag.router, prefix="/api", tags=["rag"])

    @app.on_event("startup")
    def on_startup():
        create_db_and_tables()

    return app


app = create_app()


@app.get("/health")
async def health_check():
    return {"status": "healthy"}
