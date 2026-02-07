"""Content filtering service for personalized content delivery based on user profiles."""

from typing import Any, Dict, List, Optional

from backend.models.auth import UserProfilePublic


class ContentFilteringService:
    """Service to filter and personalize content based on user profile data."""

    @staticmethod
    def filter_content_by_profile(
        content_list: List[Dict[str, Any]], user_profile: UserProfilePublic
    ) -> List[Dict[str, Any]]:
        """
        Filter content based on user profile information.

        Args:
            content_list: List of content items to filter
            user_profile: User's profile information

        Returns:
            Filtered list of content items
        """
        filtered_content = []

        for content in content_list:
            if ContentFilteringService._should_include_content(content, user_profile):
                filtered_content.append(content)

        # Apply ordering based on user profile
        ordered_content = ContentFilteringService._order_content_by_profile(
            filtered_content, user_profile
        )

        return ordered_content

    @staticmethod
    def _should_include_content(
        content: Dict[str, Any], user_profile: UserProfilePublic
    ) -> bool:
        """
        Determine if content should be included based on user profile.

        Args:
            content: Content item to evaluate
            user_profile: User's profile information

        Returns:
            True if content should be included, False otherwise
        """
        # Get content tags or categories
        content_tags = content.get("tags", [])
        content_category = content.get("category", "").lower()
        content_level = content.get("level", "intermediate").lower()

        # Apply HARDWARE_ONLY → hide AI/ML personalization rule
        if user_profile.learning_track == "HARDWARE_ONLY":
            if (
                any(
                    tag.lower()
                    in ["ai", "ml", "artificial-intelligence", "machine-learning"]
                    for tag in content_tags
                )
                or "ai" in content_category
                or "ml" in content_category
            ):
                return False

        # Apply SOFTWARE_ONLY → hide hardware content rule
        if user_profile.learning_track == "SOFTWARE_ONLY":
            if (
                any(
                    tag.lower()
                    in ["hardware", "electronics", "arduino", "raspberry-pi", "esp32"]
                    for tag in content_tags
                )
                or "hardware" in content_category
            ):
                return False

        # Apply level-based filtering
        if user_profile.software_level == "BEGINNER":
            # Beginners should not see advanced content
            if content_level == "advanced":
                return False
        elif user_profile.software_level == "ADVANCED":
            # Advanced users might skip basic content
            if (
                content_level == "beginner"
                and user_profile.software_level == "ADVANCED"
            ):
                # For now, still include beginner content as foundation
                pass

        # Apply hardware experience filtering
        if (
            user_profile.hardware_experience == "NONE"
            and user_profile.learning_track != "SOFTWARE_ONLY"
        ):
            # Users with no hardware experience might need to skip advanced hardware content
            if "advanced" in content_tags and any(
                hw_tag in content_tags for hw_tag in ["hardware", "electronics"]
            ):
                return False

        return True

    @staticmethod
    def _order_content_by_profile(
        content_list: List[Dict[str, Any]], user_profile: UserProfilePublic
    ) -> List[Dict[str, Any]]:
        """
        Order content based on user profile preferences.

        Args:
            content_list: List of content items to order
            user_profile: User's profile information

        Returns:
            Ordered list of content items
        """

        def get_content_priority(content: Dict[str, Any]) -> int:
            """Calculate priority score for content based on user profile."""
            priority = 0
            content_tags = content.get("tags", [])
            content_category = content.get("category", "").lower()
            content_level = content.get("level", "intermediate").lower()

            # BEGINNER → foundations first personalization rule
            if user_profile.software_level == "BEGINNER":
                if content_level == "beginner":
                    priority += 10  # Prioritize beginner content
                elif "foundation" in content_tags or "basics" in content_tags:
                    priority += 5  # Prioritize foundational content

            # Apply learning track preferences
            if user_profile.learning_track == "FULL_ROBOTICS":
                # Unlock ROS/control systems content
                if "ros" in content_tags or "control-systems" in content_tags:
                    priority += 15  # High priority for ROS/control systems for full robotics track
            elif user_profile.learning_track == "HARDWARE_ONLY":
                # Prioritize hardware content
                if any(
                    hw_tag in content_tags
                    for hw_tag in [
                        "hardware",
                        "electronics",
                        "arduino",
                        "raspberry-pi",
                        "esp32",
                    ]
                ):
                    priority += 10
            elif user_profile.learning_track == "SOFTWARE_ONLY":
                # Prioritize software content
                if any(
                    sw_tag in content_tags
                    for sw_tag in [
                        "software",
                        "programming",
                        "algorithm",
                        "data-structure",
                    ]
                ):
                    priority += 10

            # Apply known language preferences
            for lang in user_profile.known_languages:
                if lang.lower() in content_tags or lang.lower() in content_category:
                    priority += 5

            # Apply board preferences
            for board in user_profile.boards_used or []:
                if (
                    board.lower().replace(" ", "-") in content_tags
                    or board.lower().replace(" ", "_") in content_tags
                ):
                    priority += 3

            # Default priority for matching categories
            if user_profile.software_level.lower() in content_level:
                priority += 1

            return priority

        # Sort content by priority (descending)
        return sorted(content_list, key=get_content_priority, reverse=True)

    @staticmethod
    def get_personalized_recommendations(
        all_content: List[Dict[str, Any]],
        user_profile: UserProfilePublic,
        limit: Optional[int] = None,
    ) -> List[Dict[str, Any]]:
        """
        Get personalized content recommendations for the user.

        Args:
            all_content: Complete list of available content
            user_profile: User's profile information
            limit: Optional limit on number of recommendations

        Returns:
            List of personalized content recommendations
        """
        filtered_content = ContentFilteringService.filter_content_by_profile(
            all_content, user_profile
        )

        if limit:
            return filtered_content[:limit]

        return filtered_content


# Global instance
content_filtering_service = ContentFilteringService()
