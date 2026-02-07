# Quickstart: Frontend Chat Integration

## Prerequisites

- Node.js 18+ installed
- Docusaurus project set up
- Backend API services running (with `/query` and `/text-selection-query` endpoints)

## Setup

1. **Install dependencies**:
   ```bash
   npm install react react-dom typescript @docusaurus/core
   ```

2. **Create chat components**:
   ```bash
   mkdir -p src/components/chat
   touch src/components/chat/ChatWidget.tsx
   touch src/components/chat/ChatMessage.tsx
   touch src/components/chat/ChatInput.tsx
   ```

3. **Add CSS styling**:
   ```bash
   mkdir -p static/css
   touch static/css/chat.css
   ```

## Basic Usage

1. **Integrate the chat widget** into your Docusaurus layout by adding it to your theme or layout components.

2. **Configure API endpoints** with the base URL for your backend services.

3. **Initialize the chat session** when the widget is loaded.

## Development

1. **Run the Docusaurus development server**:
   ```bash
   npm run start
   ```

2. **Test the chat functionality** by opening the textbook pages and interacting with the chat widget.

3. **Verify API connectivity** by checking network requests to the backend endpoints.

## Testing

1. **Unit tests**: Run component tests with Jest and React Testing Library
   ```bash
   npm test
   ```

2. **End-to-end tests**: Run with Cypress
   ```bash
   npx cypress run
   ```

## Building for Production

1. **Build the Docusaurus site**:
   ```bash
   npm run build
   ```

2. **Serve the production build**:
   ```bash
   npm run serve
   ```
