# Quickstart: UI Layout, Theme & Animation Update

**Feature**: 001-ui-theme-update
**Date**: 2025-12-15

## Setup Environment

1. Ensure you're in the `docusaurus-textbook` directory:
   ```bash
   cd docusaurus-textbook
   ```

2. Install required dependencies:
   ```bash
   npm install @docusaurus/module-type-aliases @docusaurus/types
   ```

3. Install additional dependencies for the new features:
   ```bash
   npm install framer-motion
   npm install lucide-react  # For new icons
   ```

## Implementation Steps

### 1. Add Poppins Font
- Update `docusaurus.config.ts` to include Poppins font from Google Fonts
- Configure font loading in the theme

### 2. Configure Global Theme
- Update `src/css/custom.css` with the new gradient background
- Define CSS variables for the color palette
- Apply theme to base elements

### 3. Update Layout Components
- Modify the main layout wrapper in `src/theme/Layout`
- Update navbar and footer components
- Ensure responsive behavior

### 4. Create New UI Components
- Create themed buttons, cards, and typography components
- Implement animation components using Framer Motion
- Ensure accessibility compliance

### 5. Update Visual Assets
- Replace icons with Lucide React components
- Update images with new design language
- Optimize assets for performance

## Testing

1. Run the development server:
   ```bash
   npm start
   ```

2. Verify all pages render with new theme
3. Test responsive behavior on different screen sizes
4. Confirm animations work as expected
5. Validate accessibility features

## Deployment

1. Build the site:
   ```bash
   npm run build
   ```

2. Test the build locally:
   ```bash
   npm run serve
   ```

3. Deploy the `build` folder to your hosting platform
