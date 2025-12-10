from sqlmodel import SQLModel
from ..config.database import engine

# Import the models to register them with SQLModel
from ..models import ChatSession, ChatMessage


def create_db_and_tables():
    """
    Create database tables based on SQLModel definitions
    """
    print("Creating database tables...")
    SQLModel.metadata.create_all(engine)
    print("Database tables created successfully!")


if __name__ == "__main__":
    create_db_and_tables()
