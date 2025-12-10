from sqlmodel import create_engine, Session
from typing import Generator
from .settings import settings
import os


# Database URL from settings
DATABASE_URL = settings.database_url

# Create the database engine
engine = create_engine(DATABASE_URL, echo=True)


def get_session() -> Generator[Session, None, None]:
    with Session(engine) as session:
        yield session