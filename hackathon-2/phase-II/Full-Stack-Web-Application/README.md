# TASKAPP - Professional Full-Stack Todo Application

A professional full-stack todo application with Next.js frontend featuring public access without authentication, local storage persistence, dark-first theme with blue accents, and comprehensive task management features.

## Features

- 🚀 Next.js 16+ with App Router for modern web development
- 🔓 Public access - no login required to use the application
- 💾 Database integration (PostgreSQL / Neon) storage persistence for tasks and preferences
- 🎨 Dark-first theme with #000000 background, #2563EB blue accents, and #FFFFFF white text
- 🌙 Smooth dark/light theme switching with next-themes
- 📱 Responsive design for all device sizes
- ♿ WCAG 2.1 AA accessibility compliant
- 🔁 Recurring tasks with daily/weekly patterns
- 🏷️ Tag-based task categorization
- 📊 Task filtering, sorting, and search capabilities
- ✨ Smooth animations and loading states

## Tech Stack

- **Frontend**: Next.js 16+, React, TypeScript, Tailwind CSS, next-themes
- **State Management**: Database integration (PostgreSQL / Neon) with persistence
- **Animations**: Framer Motion
- **Drag & Drop**: dnd-kit
- **Styling**: Tailwind CSS with custom dark-first theme

## Prerequisites

- Node.js v20+
- Git

## Setup Instructions

1. Clone the repository
2. Install dependencies:
   ```bash
   # Frontend
   cd frontend
   npm install
   ```
3. Run the application:
   ```bash
   # Frontend
   cd frontend
   npm run dev
   ```

## Project Structure

```
monorepo root
├── .env                          # Environment variables
├── .env.example                  # Example environment variables
├── frontend/                     # Next.js frontend application
│   ├── app/                      # App Router pages
│   ├── components/               # React components
│   ├── lib/                      # Utilities and store
│   ├── styles/                   # Global styles
│   └── ...
```
