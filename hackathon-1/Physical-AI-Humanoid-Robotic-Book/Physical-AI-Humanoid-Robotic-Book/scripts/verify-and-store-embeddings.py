#!/usr/bin/env python3
"""
Script to verify if embeddings are stored in Qdrant database and store them if not present
"""

import logging
import os
import sys
from pathlib import Path

# Add the backend src directory to the path so we can import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "backend", "src"))

try:
    import cohere
    from qdrant_client import QdrantClient
    from qdrant_client.http import models
except ImportError as e:
    print(f"Required packages not found: {e}")
    print("Please install: pip install cohere qdrant-client")
    sys.exit(1)

from services.book_embedding_service import BookEmbeddingService

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def check_qdrant_connection(qdrant_url: str, qdrant_api_key: str = None):
    """
    Check if we can connect to Qdrant
    """
    try:
        client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key, prefer_grpc=False)

        # Test connection by getting collections
        collections = client.get_collections()
        logger.info(
            f"✅ Successfully connected to Qdrant. Found {len(collections.collections)} collections"
        )
        return client
    except Exception as e:
        logger.error(f"❌ Failed to connect to Qdrant: {e}")
        return None


def check_collection_exists(client, collection_name: str):
    """
    Check if the specified collection exists
    """
    try:
        collections = client.get_collections()
        collection_exists = any(
            col.name == collection_name for col in collections.collections
        )

        if collection_exists:
            logger.info(f"✅ Collection '{collection_name}' exists")
            return True
        else:
            logger.info(f"❌ Collection '{collection_name}' does not exist")
            return False
    except Exception as e:
        logger.error(f"❌ Error checking collection: {e}")
        return False


def check_collection_count(client, collection_name: str):
    """
    Check the number of embeddings in the collection
    """
    try:
        count = client.count(collection_name=collection_name)
        logger.info(f"📊 Collection '{collection_name}' has {count.count} embeddings")
        return count.count
    except Exception as e:
        logger.error(f"❌ Error getting collection count: {e}")
        return 0


def store_sample_embeddings(service):
    """
    Store sample embeddings if none exist
    """
    print("📦 Storing sample Physical AI & Humanoid Robotics embeddings...")

    # Sample content from the textbook
    sample_contents = [
        {
            "title": "Introduction to Physical AI",
            "content": """
            Physical AI is an emerging field that combines artificial intelligence with physical systems.
            It focuses on creating intelligent agents that can interact with the real world through sensors
            and actuators. Unlike traditional AI that operates in virtual environments, Physical AI systems
            must deal with the complexities and uncertainties of the physical world.

            Key characteristics of Physical AI systems include:
            - Embodied interaction with the environment
            - Real-time processing and response
            - Handling of noisy and incomplete sensor data
            - Adaptation to dynamic physical conditions
            - Safety and reliability in physical interactions

            Applications of Physical AI include robotics, autonomous vehicles, smart manufacturing,
            and human-robot collaboration. The field draws from multiple disciplines including
            machine learning, robotics, control theory, and cognitive science.
            """,
            "chapter": "Chapter 1",
            "section": "1.1",
        },
        {
            "title": "Humanoid Robotics Fundamentals",
            "content": """
            Humanoid robots are robots with physical anthropomorphic characteristics. They are designed
            to resemble and imitate human behavior, often featuring two legs, two arms, a head, and a torso.
            These robots are particularly interesting because they can operate in human-designed environments
            and potentially interact with humans in more natural ways.

            Key challenges in humanoid robotics include:
            - Balance and locomotion control
            - Human-like manipulation skills
            - Natural human-robot interaction
            - Real-time motion planning
            - Robustness in dynamic environments

            Modern humanoid robots like ASIMO, Atlas, and NAO demonstrate various capabilities in
            walking, object manipulation, and basic interaction. However, achieving human-level
            dexterity and intelligence remains a significant challenge.
            """,
            "chapter": "Chapter 2",
            "section": "2.1",
        },
        {
            "title": "Embodied Cognition",
            "content": """
            Embodied cognition is the theory that many features of cognition, whether human or otherwise,
            are shaped by aspects of the entire body of the organism. In contrast to traditional views
            that treat the body as merely an input/output device for the brain, embodied cognition
            suggests that the body plays an active role in shaping cognition.

            In the context of robotics and AI, embodied cognition principles suggest that:
            - Physical interaction with the environment is crucial for learning
            - Sensorimotor experiences form the basis of higher-level cognition
            - The body's morphology influences cognitive processes
            - Embodied agents can develop more robust and adaptive behaviors

            This approach has led to the development of morphological computation, where the physical
            properties of the body contribute to computational processes, reducing the burden on
            central processing units.
            """,
            "chapter": "Chapter 3",
            "section": "3.2",
        },
        {
            "title": "Deep Reinforcement Learning for Robotics",
            "content": """
            Deep reinforcement learning (DRL) has emerged as a powerful approach for training
            robotic systems. By combining deep neural networks with reinforcement learning,
            DRL enables robots to learn complex behaviors through trial and error in simulated
            or real environments.

            Key advantages of DRL for robotics:
            - Learning of complex, multi-step behaviors
            - Adaptation to novel situations
            - End-to-end learning from raw sensor data
            - Generalization across different tasks

            However, challenges remain:
            - Sample efficiency compared to human learning
            - Safety during exploration in real environments
            - Transfer from simulation to reality (sim-to-real gap)
            - Robustness to environmental changes

            Recent advances in domain randomization and sim-to-real transfer techniques
            have improved the applicability of DRL to real robotic systems.
            """,
            "chapter": "Chapter 4",
            "section": "4.3",
        },
        {
            "title": "Sim-to-Real Transfer",
            "content": """
            Sim-to-real transfer is the process of transferring policies learned in simulation
            to real-world robots. This approach is essential because training directly on
            physical robots is often impractical due to safety concerns, time constraints,
            and potential damage to equipment.

            The reality gap refers to the differences between simulated and real environments
            that can cause policies learned in simulation to fail when deployed on real robots.
            These differences include:
            - Visual appearance and lighting conditions
            - Physics simulation inaccuracies
            - Sensor noise and delays
            - Actuator dynamics
            - Environmental disturbances

            Techniques to bridge the reality gap include:
            - Domain randomization: training in varied simulated environments
            - System identification: creating more accurate models
            - Domain adaptation: adjusting policies for real-world conditions
            - Few-shot learning: rapid adaptation with minimal real-world data
            """,
            "chapter": "Chapter 5",
            "section": "5.1",
        },
    ]

    all_point_ids = []
    for content in sample_contents:
        print(f"   Storing: {content['title']}")
        point_ids = service.store_book_content(
            title=content["title"],
            content=content["content"],
            chapter=content["chapter"],
            section=content["section"],
        )
        all_point_ids.extend(point_ids)
        print(f"   ✅ Stored {len(point_ids)} chunks for '{content['title']}'")

    print(f"✅ Successfully stored {len(all_point_ids)} total content chunks")
    return all_point_ids


def main():
    """
    Main function to verify and store embeddings in Qdrant
    """
    print("🔍 Verifying Qdrant Vector Database Embeddings...")

    # Get environment variables
    qdrant_url = os.getenv("QDRANT_URL", "http://localhost:6333")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")
    cohere_api_key = os.getenv("COHERE_API_KEY")

    if not cohere_api_key:
        print("❌ COHERE_API_KEY environment variable is required")
        print("Please set your Cohere API key before running this script")
        return False

    # Check Qdrant connection
    print(f"📡 Connecting to Qdrant at {qdrant_url}...")
    client = check_qdrant_connection(qdrant_url, qdrant_api_key)

    if not client:
        print("❌ Cannot connect to Qdrant. Please check your connection settings.")
        return False

    # Check collections
    collection_name = "book_embeddings"
    collection_exists = check_collection_exists(client, collection_name)

    # Check count
    current_count = check_collection_count(client, collection_name)

    if current_count > 0:
        print(
            f"✅ Embeddings are already stored in Qdrant! Found {current_count} embeddings in '{collection_name}' collection."
        )

        # Show some sample data
        try:
            points = client.scroll(
                collection_name=collection_name, limit=3, with_payload=True
            )

            print(f"\n📋 Sample embeddings in the database:")
            for i, (point, _) in enumerate(points, 1):
                title = point.payload.get("title", "Unknown")
                content_preview = point.payload.get("content", "")[:100] + "..."
                print(f"  {i}. Title: {title}")
                print(f"     Content: {content_preview}")
        except Exception as e:
            print(f"⚠️ Could not retrieve sample data: {e}")

        return True
    else:
        print(f"❌ No embeddings found in '{collection_name}' collection.")

        # Initialize the book embedding service
        try:
            service = BookEmbeddingService()
            print("✅ Book embedding service initialized")
        except Exception as e:
            logger.error(f"❌ Error initializing book embedding service: {e}")
            return False

        # Store sample embeddings
        print("\n📦 Storing embeddings in Qdrant vector database...")
        point_ids = store_sample_embeddings(service)

        # Verify the storage
        final_count = check_collection_count(client, collection_name)
        print(f"\n✅ Verification: Database now contains {final_count} embeddings")

        if final_count > 0:
            print("🎉 Qdrant vector database successfully populated with embeddings!")
            return True
        else:
            print("❌ Failed to store embeddings in Qdrant database")
            return False


if __name__ == "__main__":
    success = main()

    if success:
        print("\n✅ Qdrant embedding verification and storage completed successfully!")
    else:
        print("\n❌ Qdrant embedding verification and storage failed!")
        sys.exit(1)
