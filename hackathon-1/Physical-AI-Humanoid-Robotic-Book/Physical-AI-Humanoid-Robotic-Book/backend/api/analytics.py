"""Analytics API endpoints for the Book Intelligence Agent."""

from typing import List, Optional

from fastapi import APIRouter, HTTPException

from backend.models.conversation import UserAnalytics
from backend.services.conversation_service import ConversationService

router = APIRouter(prefix="/api/analytics", tags=["analytics"])


@router.post("/track", response_model=UserAnalytics)
async def track_user_analytics(
    user_id: Optional[str] = None,
    session_id: str = "",
    query: str = "",
    response_time: float = 0.0,
    was_answered: bool = False,
    used_selected_snippet: bool = False,
    satisfaction_score: Optional[int] = None,
    was_accurate: Optional[bool] = None,
):
    """Track user analytics with upsert logic."""
    try:
        service = ConversationService()
        analytics = await service.create_or_update_user_analytics(
            user_id=user_id,
            session_id=session_id,
            query=query,
            response_time=response_time,
            was_answered=was_answered,
            used_selected_snippet=used_selected_snippet,
            satisfaction_score=satisfaction_score,
            was_accurate=was_accurate,
        )
        return analytics
    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Failed to track user analytics: {str(e)}"
        )


@router.get("/user/{user_id}", response_model=List[UserAnalytics])
async def get_user_analytics(user_id: str):
    """Retrieve user analytics by user ID."""
    try:
        service = ConversationService()
        analytics_list = await service.get_user_analytics(user_id)
        return analytics_list
    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Failed to retrieve user analytics: {str(e)}"
        )
