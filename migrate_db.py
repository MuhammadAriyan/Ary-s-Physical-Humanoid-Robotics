#!/usr/bin/env python3
"""
Database migration script to add the 'content' column to the document_metadata table.
"""

import os
from sqlalchemy import create_engine, text
from backend.app.config.settings import settings

def migrate_database():
    """Add the content column to the document_metadata table."""
    
    # Get the database URL from settings
    database_url = settings.database_url
    
    print(f"Connecting to database: {database_url}")
    
    # Create engine
    engine = create_engine(database_url)
    
    # Check if the content column already exists
    with engine.connect() as conn:
        # Check if 'content' column exists in the document_metadata table
        result = conn.execute(text("""
            SELECT column_name 
            FROM information_schema.columns 
            WHERE table_name = 'document_metadata' AND column_name = 'content'
        """))
        
        if result.fetchone():
            print("✓ The 'content' column already exists in the document_metadata table")
            return True
        else:
            print("The 'content' column does not exist. Adding it now...")
            
            # Add the content column
            try:
                conn.execute(text("ALTER TABLE document_metadata ADD COLUMN content TEXT"))
                conn.commit()
                print("✓ Successfully added the 'content' column to document_metadata table")
                return True
            except Exception as e:
                print(f"✗ Failed to add the 'content' column: {e}")
                conn.rollback()
                return False

if __name__ == "__main__":
    migrate_database()