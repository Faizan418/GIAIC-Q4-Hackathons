# TASKAPP Quickstart Guide

## 1. Prerequisites

### System Requirements
- Node.js v18+ (recommended v18.17.0 or later)
- npm v8+ or yarn v1.22+
- Git v2.0+
- PostgreSQL-compatible database (Neon recommended)

### Development Tools
- Code editor (VS Code recommended)
- Terminal/shell
- Browser (Chrome, Firefox, Safari, or Edge)

## 2. Setup Instructions

### 2.1 Clone the Repository
```bash
git clone <repository-url>
cd <repository-directory>
```

### 2.2 Install Dependencies
```bash
# Navigate to the project root
cd frontend  # or wherever package.json is located

# Install dependencies
npm install
# or if using yarn
yarn install
```

### 2.3 Database Setup

#### Option 1: Neon Database (Recommended)
1. Create a free Neon account at [neon.tech](https://neon.tech)
2. Create a new project
3. Copy the connection string from the Neon dashboard
4. Add to your `.env` file as `DATABASE_URL`

#### Option 2: Local PostgreSQL
1. Install PostgreSQL locally
2. Create a database for the project
3. Update connection string in `.env`

### 2.4 Environment Configuration
Create a `.env` file in the project root:
```env
DATABASE_URL="your-neon-or-postgres-connection-string"
NEXT_PUBLIC_BASE_URL="http://localhost:3000"
```

### 2.5 Database Initialization
```bash
# Run database migrations (if using Prisma)
npx prisma db push
# or for the first time
npx prisma migrate dev

# Generate Prisma client
npx prisma generate
```

## 3. Development Workflow

### 3.1 Running the Development Server
```bash
# Start the development server
npm run dev
# or
yarn dev

# The application will be available at http://localhost:3000
```

### 3.2 Running Database Migrations
```bash
# Apply pending migrations
npx prisma migrate deploy

# Create a new migration
npx prisma migrate dev --name "migration-name"
```

### 3.3 Building for Production
```bash
# Build the application
npm run build
# or
yarn build

# Run the production build
npm run start
# or
yarn start
```

## 4. Key Scripts

### 4.1 Package.json Scripts
```json
{
  "dev": "next dev",
  "build": "next build",
  "start": "next start",
  "lint": "next lint",
  "db:migrate": "prisma migrate dev",
  "db:push": "prisma db push",
  "db:studio": "prisma studio",
  "type-check": "tsc --noEmit"
}
```

### 4.2 Common Development Commands
```bash
# Run type checking
npm run type-check

# Run linting
npm run lint

# Run database studio to view data
npx prisma studio

# Format code (if prettier is configured)
npm run format
```

## 5. Project Structure

### 5.1 Key Directories
```
project-root/
├── frontend/                 # Next.js application
│   ├── app/                  # App Router pages
│   │   ├── tasks/           # Tasks page
│   │   ├── dashboard/       # Dashboard page
│   │   └── layout.tsx       # Root layout
│   ├── components/          # Reusable components
│   ├── lib/                 # Utility functions
│   ├── types/               # TypeScript definitions
│   └── styles/              # Global styles
├── prisma/                  # Database schema and migrations
├── public/                  # Static assets
└── package.json            # Project dependencies
```

### 5.2 Important Files
- `prisma/schema.prisma` - Database schema definition
- `app/layout.tsx` - Root layout with theme providers
- `app/tasks/page.tsx` - Main tasks page
- `app/dashboard/page.tsx` - Dashboard page
- `components/ui/` - Reusable UI components

## 6. Database Schema

### 6.1 Tasks Table
The application uses a `tasks` table with the following structure:
- `id`: UUID primary key
- `title`: TEXT (required, 1-200 chars)
- `description`: TEXT (optional)
- `completed`: BOOLEAN (default: false)
- `priority`: TEXT enum ['low', 'medium', 'high'] (default: 'medium')
- `category`: TEXT (optional)
- `type`: TEXT enum ['daily', 'weekly', 'monthly'] (required)
- `dueDate`: TIMESTAMP (optional)
- `createdAt`: TIMESTAMP (default: now)
- `updatedAt`: TIMESTAMP (default: now, updates on change)
- `completedAt`: TIMESTAMP (optional, set when completed)

## 7. API Endpoints

### 7.1 Available Endpoints
- `GET /api/tasks` - Get all tasks
- `POST /api/tasks` - Create a new task
- `PUT /api/tasks/:id` - Update a task
- `DELETE /api/tasks/:id` - Delete a task

### 7.2 API Response Format
Successful responses follow this pattern:
```json
{
  "data": { /* task object or array */ }
}
```

Error responses follow this pattern:
```json
{
  "error": {
    "code": "ERROR_CODE",
    "message": "Error message",
    "details": { /* optional error details */ }
  }
}
```

## 8. Theme Configuration

### 8.1 Dark/Light Theme
The application supports both dark and light themes:
- Dark mode: #000000 background, #FFFFFF text, #2563EB blue accents
- Light mode: #FFFFFF background, #000000 text, #2563EB blue accents
- Theme toggle available in the UI
- System preference detection

### 8.2 Theme Usage
Themes are managed using `next-themes`:
```tsx
import { useTheme } from "next-themes"

const { theme, setTheme } = useTheme()
```

## 9. Troubleshooting

### 9.1 Common Issues

#### Database Connection
- Verify `DATABASE_URL` in `.env` file
- Check that the database service is running
- Ensure the connection string format is correct

#### Build Errors
- Run `npm run type-check` to check for TypeScript errors
- Verify all dependencies are installed with `npm install`
- Check that environment variables are properly configured

#### Development Server Issues
- Clear Next.js cache: `rm -rf .next`
- Verify port 3000 is not in use
- Check for conflicting processes

### 9.2 Useful Commands
```bash
# Clear Next.js cache
rm -rf .next

# Reset node_modules
rm -rf node_modules
npm install

# Check environment variables
node -e "console.log(process.env.DATABASE_URL)"

# Verify database connection
npx prisma db pull
```

## 10. Next Steps

### 10.1 Development
1. Start the development server: `npm run dev`
2. Navigate to the `/tasks` page to implement CRUD functionality
3. Add new features following the established patterns
4. Test thoroughly before committing changes

### 10.2 Production Deployment
1. Ensure environment variables are set in production
2. Run `npm run build` to create a production build
3. Deploy to your preferred hosting platform
4. Monitor application performance and errors