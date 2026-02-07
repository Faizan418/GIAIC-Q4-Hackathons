"""
Middleware for the Todo API
Includes logging, request timing, and other cross-cutting concerns
"""

import time
import logging
from fastapi import Request
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import Response


class LoggingMiddleware(BaseHTTPMiddleware):
    async def dispatch(self, request: Request, call_next):
        start_time = time.time()

        # Log incoming request
        logging.info(f"Request: {request.method} {request.url}")

        try:
            response = await call_next(request)
        except Exception as e:
            # Log exceptions
            logging.error(f"Exception during request {request.method} {request.url}: {str(e)}")
            raise

        # Calculate request duration
        duration = time.time() - start_time

        # Add security headers
        response.headers["X-Content-Type-Options"] = "nosniff"
        response.headers["X-Frame-Options"] = "DENY"
        response.headers["X-XSS-Protection"] = "1; mode=block"

        # Add cache control headers to prevent browser caching of API responses
        response.headers["Cache-Control"] = "no-store, no-cache, must-revalidate"
        response.headers["Pragma"] = "no-cache"
        response.headers["Expires"] = "0"

        # Log response
        logging.info(f"Response: {response.status_code} in {duration:.2f}s")

        return response


# Set up basic logging configuration
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')