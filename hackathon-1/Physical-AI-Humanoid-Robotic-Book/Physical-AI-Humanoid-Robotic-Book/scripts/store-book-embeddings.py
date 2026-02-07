#!/usr/bin/env python3
"""
Script to store Physical AI & Humanoid Robotics textbook content as embeddings in Qdrant
This script processes the existing documentation and stores it in the vector database
"""

import logging
import os
import sys
from pathlib import Path

# Add the backend src directory to the path so we can import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "backend", "src"))

from services.book_embedding_service import BookEmbeddingService

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def store_textbook_embeddings():
    """
    Store textbook content as embeddings in Qdrant
    """
    print("📚 Starting to store Physical AI & Humanoid Robotics textbook embeddings...")

    try:
        # Initialize the book embedding service
        service = BookEmbeddingService()
        print("✅ Book embedding service initialized successfully")

        # Define the documentation directory
        docs_dir = Path("../docusaurus-textbook/docs")
        if not docs_dir.exists():
            docs_dir = Path("docusaurus-textbook/docs")

        if not docs_dir.exists():
            docs_dir = Path(
                "/mnt/e/Hackathon 1/Physical-AI-Humanoid-Robotic-Book/docusaurus-textbook/docs"
            )

        if not docs_dir.exists():
            print(f"❌ Documentation directory not found at any expected location")
            return False

        print(f"📖 Processing textbook content from: {docs_dir}")

        # Store all markdown files from the documentation
        results = service.store_book_from_directory(
            str(docs_dir), title_prefix="Physical AI & Humanoid Robotics Textbook"
        )

        # Print summary of what was stored
        total_chunks = 0
        for file_path, chunk_ids in results.items():
            print(f"  📄 {Path(file_path).name}: {len(chunk_ids)} chunks")
            total_chunks += len(chunk_ids)

        print(f"\n✅ Successfully stored {total_chunks} content chunks from textbook")

        # Verify the storage by checking the count
        count = service.get_book_embedding_count()
        print(f"📊 Total book embeddings in database: {count}")

        # Get unique books/chapters in the collection
        books = service.get_books_in_collection()
        print(f"📚 Unique titles in collection: {len(books)}")

        # Show a few examples
        for i, book in enumerate(books[:5]):
            print(f"   - {book}")
        if len(books) > 5:
            print(f"   ... and {len(books) - 5} more")

        # Test search functionality with a sample query
        print(f"\n🔍 Testing search functionality...")
        search_results = service.search_book_content("humanoid robotics", limit=3)

        print(f"Sample search results for 'humanoid robotics':")
        for i, result in enumerate(search_results, 1):
            print(f"  {i}. Score: {result['score']:.3f}")
            print(f"     Title: {result['title']}")
            print(f"     Content: {result['content'][:100]}...")
            print()

        print("🎉 Textbook embeddings successfully stored in Qdrant vector database!")
        return True

    except Exception as e:
        logger.error(f"❌ Error storing textbook embeddings: {e}")
        return False


def store_sample_content():
    """
    Store sample textbook content if documentation is not available
    """
    print("📚 Storing sample Physical AI & Humanoid Robotics content...")

    try:
        # Initialize the book embedding service
        service = BookEmbeddingService()
        print("✅ Book embedding service initialized successfully")

        # Sample content from a typical Physical AI & Humanoid Robotics textbook
        textbook_sections = [
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

        # Store each section
        all_point_ids = []
        for section in textbook_sections:
            print(f"📦 Storing: {section['title']}")
            point_ids = service.store_book_content(
                title=section["title"],
                content=section["content"],
                chapter=section["chapter"],
                section=section["section"],
            )
            all_point_ids.extend(point_ids)
            print(f"   ✅ Stored {len(point_ids)} chunks")

        print(f"\n✅ Successfully stored {len(all_point_ids)} total content chunks")

        # Verify the storage
        count = service.get_book_embedding_count()
        print(f"📊 Total book embeddings in database: {count}")

        # Test search functionality
        print(f"\n🔍 Testing search functionality...")

        # Test various search queries
        queries = [
            "humanoid robotics",
            "embodied cognition",
            "reinforcement learning robotics",
            "sim-to-real transfer",
        ]

        for query in queries:
            search_results = service.search_book_content(query, limit=2)
            print(f"Search results for '{query}':")
            for i, result in enumerate(search_results, 1):
                print(f"  {i}. Score: {result['score']:.3f}")
                print(f"     Title: {result['title']}")
                print(f"     Content: {result['content'][:100]}...")
            print()

        print(
            "🎉 Sample textbook embeddings successfully stored in Qdrant vector database!"
        )
        return True

    except Exception as e:
        logger.error(f"❌ Error storing sample content: {e}")
        return False


def main():
    """
    Main function to store book embeddings
    """
    print("🚀 Starting Book Embedding Storage Process...")
    print(
        "This will store Physical AI & Humanoid Robotics textbook content in Qdrant vector database."
    )

    # Try to store from actual documentation first, fall back to sample content
    success = store_textbook_embeddings()

    if not success:
        print("\n⚠️  Documentation not found, storing sample content instead...")
        success = store_sample_content()

    if success:
        print(f"\n🎉 Book embedding storage completed successfully!")
        print(
            "The Physical AI & Humanoid Robotics textbook content is now stored in Qdrant vector database."
        )
        print(
            "You can now perform semantic searches on the textbook content using the embedding service."
        )
    else:
        print(f"\n❌ Failed to store book embeddings.")
        return False

    return True


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
