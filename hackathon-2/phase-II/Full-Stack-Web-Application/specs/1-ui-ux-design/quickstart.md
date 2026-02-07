# Quickstart Guide: TASKAPP Frontend

## Development Setup

### Prerequisites
- Node.js 18+ installed
- npm or yarn package manager
- Git for version control

### Initial Setup

1. **Clone the repository** (if not already done):
```bash
git clone <repository-url>
cd Full-Stack-Web-Application
cd frontend
```

2. **Install dependencies**:
```bash
npm install
# or
yarn install
```

3. **Set up environment variables**:
Create a `.env.local` file in the frontend directory with the following:
```env
NEXT_PUBLIC_API_BASE_URL=http://localhost:8000
NEXT_PUBLIC_BETTER_AUTH_URL=http://localhost:8000
BETTER_AUTH_URL=http://localhost:8000
BETTER_AUTH_SECRET=<your-secret-key>
```

4. **Run the development server**:
```bash
npm run dev
# or
yarn dev
```

5. **Open your browser** to `http://localhost:3000` to view the application.

## Key Commands

### Development
```bash
npm run dev          # Start development server with hot reloading
npm run build        # Build the application for production
npm run start        # Start production server (after building)
npm run lint         # Check code for linting errors
npm run format       # Format code with Prettier
```

### Testing
```bash
npm test            # Run unit tests
npm run test:watch  # Run tests in watch mode
npm run test:coverage  # Run tests with coverage report
```

## Project Structure

```
frontend/
├── app/                    # Next.js App Router pages
│   ├── layout.tsx         # Root layout with theme provider
│   ├── page.tsx           # Landing page
│   ├── dashboard/         # Dashboard route
│   ├── api/              # API routes
│   └── auth/             # Authentication routes
├── components/           # Reusable UI components
│   ├── ui/              # Base components (Button, Card, etc.)
│   ├── auth/            # Authentication components
│   ├── dashboard/       # Dashboard components
│   └── landing/         # Landing page components
├── lib/                # Utility functions and API clients
│   ├── api.ts          # API client implementation
│   ├── auth-client.ts  # Better Auth client
│   └── utils.ts        # General utility functions
├── styles/             # Global styles and themes
│   └── globals.css     # Global CSS and Tailwind imports
├── public/             # Static assets
├── types/              # TypeScript type definitions
└── middleware.ts       # Next.js middleware
```

## Running with Backend

To run the full application with both frontend and backend:

1. **Start the backend server** (in a separate terminal):
```bash
cd ../backend
pip install -r requirements.txt
uvicorn main:app --reload
```

2. **Ensure frontend environment variables** point to the backend:
```env
NEXT_PUBLIC_API_BASE_URL=http://localhost:8000
NEXT_PUBLIC_BETTER_AUTH_URL=http://localhost:8000
```

## Common Tasks

### Adding a New Page
1. Create a new directory in `app/` with the route name
2. Add a `page.tsx` file in that directory
3. Export your component as the default export

### Creating a New Component
1. Create a new file in the appropriate subdirectory of `components/`
2. Use PascalCase for component names
3. Export the component as the default export
4. Add proper TypeScript typing

### Adding API Calls
1. Update the API client in `lib/api.ts`
2. Add proper TypeScript interfaces for request/response types
3. Handle loading and error states appropriately

### Theming and Styling
1. Update the theme configuration in `tailwind.config.js`
2. Add custom CSS in `styles/globals.css`
3. Use Tailwind classes for styling components

## Troubleshooting

### Common Issues

**Issue**: "Module not found" errors
**Solution**: Run `npm install` to ensure all dependencies are installed

**Issue**: Authentication not working
**Solution**: Verify that BETTER_AUTH_SECRET is set correctly in both frontend and backend

**Issue**: API calls failing
**Solution**: Check that NEXT_PUBLIC_API_BASE_URL points to the running backend server

**Issue**: Hot reloading not working
**Solution**: Restart the development server with `npm run dev`

**Issue**: Styles not applying
**Solution**: Verify Tailwind is properly configured and classes are correctly applied

### Environment Configuration
If you're having issues with environment variables:
1. Verify `.env.local` file exists in the frontend directory
2. Check that variables are prefixed with `NEXT_PUBLIC_` for client-side access
3. Restart the development server after changing environment variables

## Deployment

### Local Build Test
```bash
npm run build && npm run start
```

### Vercel Deployment
The application is configured for easy deployment to Vercel:
1. Connect your GitHub repository to Vercel
2. Add the required environment variables in Vercel dashboard
3. Deploy automatically on pushes to main branch

### Environment Variables for Production
Ensure these variables are set in your production environment:
```env
NEXT_PUBLIC_API_BASE_URL=https://your-backend-domain.com
NEXT_PUBLIC_BETTER_AUTH_URL=https://your-backend-domain.com
NODE_ENV=production
```

## Next Steps

1. Review the UI/UX specification in `specs/1-ui-ux-design/spec.md`
2. Check the implementation plan in `specs/1-ui-ux-design/plan.md`
3. Look at the data model in `specs/1-ui-ux-design/data-model.md`
4. Review API contracts in `specs/1-ui-ux-design/contracts/`
5. Start implementing components following the design system