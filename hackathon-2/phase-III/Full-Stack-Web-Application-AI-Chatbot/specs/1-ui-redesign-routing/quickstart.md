# Quick Start Guide: TASKAPP Public Access Version

**Feature**: 1-ui-redesign-routing
**Created**: 2026-01-16

## Overview

This guide explains how to set up, run, and use TASKAPP with public access and the new dark-first UI design. The application no longer requires authentication, allowing immediate access to all features.

## Prerequisites

- Node.js 18+ (for frontend)
- Python 3.9+ (for backend)
- npm or yarn package manager
- Git for version control

## Setup Instructions

### 1. Clone and Navigate to Repository
```bash
git clone [repository-url]
cd Full-Stack-Web-Application
```

### 2. Set Up Frontend (Next.js)

Navigate to the frontend directory:
```bash
cd frontend
```

Install dependencies:
```bash
npm install
# or
yarn install
```

Create a `.env.local` file in the frontend directory with the following:
```env
NEXT_PUBLIC_API_BASE_URL=http://localhost:8000
BACKEND_URL=http://localhost:8000
```

### 3. Set Up Backend (FastAPI)

Open a new terminal and navigate to the backend directory:
```bash
cd backend
```

Set up Python virtual environment:
```bash
python -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
```

Install dependencies:
```bash
pip install -r requirements.txt
```

Create a `.env` file in the backend directory with the following:
```env
DATABASE_URL=sqlite:///./taskapp_dev.db
BETTER_AUTH_SECRET=your-super-secret-jwt-key-here-make-it-long-and-random
```

### 4. Run the Applications

#### Start the Backend
From the backend directory:
```bash
python -m src.main
# or
uvicorn src.main:app --reload --port 8000
```

#### Start the Frontend
From the frontend directory:
```bash
npm run dev
# or
yarn dev
```

The application will be available at `http://localhost:3000`

## Using TASKAPP (Public Access Version)

### Accessing the Application
- Navigate to `http://localhost:3000`
- No login required - access all features immediately
- The application loads with dark theme by default

### Main Features

#### 1. Navigation
- **TASKAPP** (logo/text) in the top-left corner
- **Dashboard** link - main application view
- **Tasks** link - view all tasks
- **Theme Toggle** - switch between dark/light modes

#### 2. Task Management
- **View Tasks**: See all tasks in the local workspace
- **Create Task**: Click "Create Task" button to add new tasks
- **Edit Task**: Click on a task to modify its details
- **Delete Task**: Remove tasks using the delete button
- **Toggle Completion**: Check/uncheck the completion box

#### 3. Task Properties
When creating or editing tasks, you can set:
- **Title**: Required (1-100 characters)
- **Description**: Optional (up to 5000 characters)
- **Priority**: High, Medium, or Low
- **Tags**: Add tags to categorize tasks
- **Due Date**: Set a deadline for the task
- **Recurrence**: Set to repeat daily or weekly

### Local Data Persistence
- Tasks are stored locally in your browser's storage
- Data persists between sessions
- Data is cleared when browser storage is cleared
- Data is not shared between different browsers or devices

## Development

### Running in Development Mode
Frontend:
```bash
npm run dev
```

Backend:
```bash
uvicorn src.main:app --reload --port 8000
```

### Building for Production
Frontend:
```bash
npm run build
```

The build must complete successfully without authentication-related errors.

### Environment Variables

#### Frontend (`.env.local`)
```env
NEXT_PUBLIC_API_BASE_URL=http://localhost:8000
BACKEND_URL=http://localhost:8000
```

#### Backend (`.env`)
```env
DATABASE_URL=sqlite:///./taskapp_dev.db
BETTER_AUTH_SECRET=your-super-secret-jwt-key-here-make-it-long-and-random
```

## Troubleshooting

### Common Issues

#### Application Won't Start
- Ensure both frontend and backend dependencies are installed
- Check that ports 3000 and 8000 are available
- Verify environment variables are set correctly

#### Build Failures
- Run `npm run build` to identify specific issues
- Ensure no authentication-related imports or logic remain in public pages
- Check for any deprecated Next.js patterns

#### Theme Not Applying
- Verify that Tailwind CSS is properly configured
- Check that CSS variables are correctly set in globals.css
- Ensure theme toggle functionality is working

#### Task Data Not Persisting
- Check browser storage permissions
- Verify local storage is not disabled
- Clear browser data and restart the application

### Verification Steps

1. **Check Public Access**: Visit all routes without authentication
   - `/` - Landing page
   - `/dashboard` - Dashboard
   - `/tasks` - Task list
   - `/tasks/create` - Task creation

2. **Verify Dark Theme**: Confirm dark-first design is applied:
   - Background: Pure black (#000000)
   - Accents: Blue (#2563EB)
   - Text: White (#FFFFFF)

3. **Test Task Features**: Ensure all task operations work:
   - Create new tasks
   - Edit existing tasks
   - Delete tasks
   - Toggle completion status

4. **Confirm No Auth Blocks**: Verify no authentication prompts or redirects

## Deployment

### Frontend Deployment
The application is designed to work with platforms like Vercel:
1. Connect your repository to Vercel
2. Set environment variables in the deployment settings
3. The build process should complete without errors

### Backend Deployment
Deploy to platforms like Render, Railway, or Heroku:
1. Set up database connection (Neon PostgreSQL recommended)
2. Configure environment variables
3. Ensure the API is accessible to the frontend

## Support

For issues with the public access version:
- Verify all authentication middleware has been removed
- Check that all routes are accessible without login
- Ensure the dark theme is properly applied
- Confirm task management features work without authentication