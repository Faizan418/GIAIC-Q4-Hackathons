# Quickstart Guide: Frontend Todo App

## Prerequisites

- Node.js v18 or higher
- npm or yarn package manager
- Git for version control
- A modern web browser for testing

## Getting Started

### 1. Clone the Repository

```bash
git clone <repository-url>
cd hackathon-todo/frontend
```

### 2. Install Dependencies

```bash
npm install
# or
yarn install
```

### 3. Environment Configuration

Create a `.env.local` file in the frontend directory with the following variables:

```env
NEXT_PUBLIC_API_BASE_URL=http://localhost:3000/api
NEXT_PUBLIC_BETTER_AUTH_URL=http://localhost:3000
BETTER_AUTH_SECRET=your-secret-key-here
DATABASE_URL=your-database-url-here
```

### 4. Run the Development Server

```bash
npm run dev
# or
yarn dev
```

The application will be available at http://localhost:3000

## Project Structure

```
frontend/
├── app/                    # Next.js App Router pages
│   ├── (auth)/            # Authentication routes (login, register)
│   ├── dashboard/         # Main dashboard page
│   ├── tasks/[id]/        # Dynamic task detail page
│   ├── layout.tsx         # Root layout with auth protection
│   └── page.tsx           # Home redirect page
├── components/            # Reusable React components
│   ├── ui/                # Primitive UI components
│   ├── task/              # Task-specific components
│   ├── filters/           # Filter and search components
│   └── feedback/          # Feedback components (toasts, loaders)
├── lib/                   # Utility functions and libraries
│   ├── api/               # API client and interceptors
│   ├── auth/              # Authentication utilities
│   └── hooks/             # Custom React hooks
├── hooks/                 # Additional custom hooks
├── types/                 # TypeScript type definitions
├── styles/                # Global styles and Tailwind config
├── public/                # Static assets
├── CLAUDE.md              # Claude Code instructions
├── next.config.js         # Next.js configuration
├── tailwind.config.js     # Tailwind CSS configuration
├── tsconfig.json          # TypeScript configuration
└── package.json           # Project dependencies and scripts
```

## Available Scripts

- `npm run dev` - Start development server with hot reloading
- `npm run build` - Build the application for production
- `npm run start` - Start production server
- `npm run lint` - Run ESLint to check for code issues
- `npm run type-check` - Run TypeScript compiler to check for type issues
- `npm test` - Run unit tests (if configured)

## Development Workflow

### Creating New Components

1. Determine the appropriate category in `components/`:
   - `ui/` for primitive components (Button, Input, Modal)
   - `task/` for task-specific components (TaskCard, TaskForm)
   - `filters/` for search/filter components
   - `feedback/` for feedback components (Toast, Loader)

2. Create a new directory with the component name:
   ```
   components/ui/Button/
   ├── Button.tsx
   ├── Button.types.ts
   └── Button.test.tsx
   ```

3. Follow the atomic design pattern and ensure proper TypeScript typing.

### Working with Forms

1. Use React Hook Form for form handling
2. Validate with Zod schemas
3. Follow the pattern established in existing forms

Example form pattern:
```typescript
import { useForm } from 'react-hook-form';
import { zodResolver } from '@hookform/resolvers/zod';
import { taskSchema, type TaskFormValues } from '@/types/task';

const TaskForm = () => {
  const form = useForm<TaskFormValues>({
    resolver: zodResolver(taskSchema),
    defaultValues: {
      title: '',
      description: '',
      priority: 'medium',
      // ... other fields
    }
  });

  // ... form implementation
};
```

### API Integration

1. API calls should be made through the client in `lib/api/`
2. JWT tokens are automatically attached to requests
3. Handle loading and error states appropriately
4. Implement optimistic updates where appropriate

### Authentication Flow

1. Public routes (login, register) are in `(auth)/` group
2. Protected routes check for valid session in layout
3. Redirect unauthenticated users to login page
4. Session is managed by Better Auth

## Styling Guidelines

### Color Palette
- Primary: Blue (#007BFF)
- Success: Dark Green (#006400)
- Background: Black (#000000)
- Text: White (for contrast)

### Typography
- Use Inter font family (imported in globals.css)
- Maintain consistent heading hierarchy (h1, h2, h3, etc.)
- Ensure adequate contrast ratios for accessibility

### Spacing
- Use Tailwind's spacing scale (0, 1, 2, 3, 4, 5, 6, 8, 10, 12, 16, 20, etc.)
- Maintain consistent padding and margin across components

## Testing Guidelines

### Component Testing
- Test UI components with React Testing Library
- Verify user interactions and state changes
- Test accessibility attributes

### Form Testing
- Test form validation with various inputs
- Verify error messages display correctly
- Test successful submission flow

### Integration Testing
- Test complete user workflows
- Verify API interactions work as expected
- Test authentication flows

## Deployment

### Building for Production
```bash
npm run build
```

### Environment Variables for Production
Ensure the following environment variables are set in your production environment:
- `NEXT_PUBLIC_API_BASE_URL` - Base URL for API calls
- `BETTER_AUTH_SECRET` - Secret for JWT signing
- `DATABASE_URL` - Connection string for database

### Platform-Specific Deployments

#### Vercel (Recommended)
1. Connect your GitHub repository to Vercel
2. Set environment variables in Vercel dashboard
3. Deploy automatically on push to main branch

#### Other Platforms
Configure environment variables according to your platform's documentation and ensure Node.js v18+ is used.

## Troubleshooting

### Common Issues

**Issue**: Authentication not working in development
**Solution**: Ensure `BETTER_AUTH_URL` is set correctly and matches your development URL

**Issue**: API calls returning 401 errors
**Solution**: Check that JWT tokens are being sent correctly in the Authorization header

**Issue**: Styles not applying correctly
**Solution**: Verify Tailwind CSS is properly configured and CSS files are imported

**Issue**: Form validation not working
**Solution**: Ensure Zod schemas are correctly defined and React Hook Form is properly configured

### Development Tips

1. Use the VS Code extensions for React, TypeScript, and Tailwind CSS for better development experience
2. Enable TypeScript checking in your editor for better type safety
3. Use the React Developer Tools browser extension for debugging component state
4. Regularly run `npm run lint` and `npm run type-check` to catch issues early

## Next Steps

1. Review the existing components to understand the code patterns
2. Check the API contracts to understand available endpoints
3. Familiarize yourself with the data models
4. Look at the existing tests to understand testing patterns
5. Review the CLAUDE.md file for specific instructions when using Claude Code for development