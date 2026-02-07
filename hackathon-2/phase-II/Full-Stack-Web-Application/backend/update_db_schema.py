"""
Update database schema to add missing name column to user table
"""
import sqlite3
import os

def update_db_schema():
    db_path = "taskapp_dev.db"

    if not os.path.exists(db_path):
        print("Database file does not exist!")
        return

    print("Updating database schema to add name column to user table...")

    try:
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()

        # Check if name column exists
        cursor.execute("PRAGMA table_info(user)")
        columns = [col[1] for col in cursor.fetchall()]

        if 'name' not in columns:
            print("Adding 'name' column to user table...")
            cursor.execute("ALTER TABLE user ADD COLUMN name VARCHAR(100)")
            print("[OK] Name column added successfully")
        else:
            print("'name' column already exists")

        # Verify the change
        cursor.execute("PRAGMA table_info(user)")
        user_columns = cursor.fetchall()
        print("Updated User table columns:")
        for col in user_columns:
            print(f"  - {col[1]} ({col[2]}) - {'' if col[5] == 0 else 'PRIMARY KEY'}")

        conn.commit()
        conn.close()
        print("Database schema updated successfully!")

    except Exception as e:
        print(f"Error updating database: {e}")

if __name__ == "__main__":
    update_db_schema()