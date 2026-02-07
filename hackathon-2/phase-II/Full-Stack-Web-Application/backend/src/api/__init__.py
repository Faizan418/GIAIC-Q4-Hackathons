# Export all API route modules
from . import task_routes
from . import auth_routes
from . import dashboard_routes
from . import public_task_routes

# Make them available for import
__all__ = ['task_routes', 'auth_routes', 'dashboard_routes', 'public_task_routes']