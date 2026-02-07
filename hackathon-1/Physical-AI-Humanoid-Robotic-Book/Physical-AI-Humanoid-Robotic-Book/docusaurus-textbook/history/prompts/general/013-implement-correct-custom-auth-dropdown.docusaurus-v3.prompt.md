---
id: 013
title: "Implement Correct Custom Auth Dropdown for Docusaurus v3"
stage: "implementation"
date: "2025-12-20"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "general"
branch: "007-user-auth-better-auth"
user: "claude"
command: "npx docusaurus start"
labels: ["auth", "navbar", "component", "docusaurus", "v3"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - "docusaurus.config.ts"
  - "src/components/AuthDropdown.tsx"
  - "src/theme/NavbarItem/Custom.tsx"
tests: []
---

## Outcome
Successfully implemented the correct custom auth dropdown solution for Docusaurus v3 following official standards.

## Evaluation
The implementation now follows Docusaurus v3 standards with the proper "custom" type and component structure.

## Files Modified
- `docusaurus.config.ts` - Updated to use type: 'custom'
- `src/components/AuthDropdown.tsx` - Created the auth dropdown component
- `src/theme/NavbarItem/Custom.tsx` - Created the required custom navbar item component

## Changes Made
1. Updated navbar config to use type: 'custom' (the only supported method in Docusaurus v3)
2. Created AuthDropdown component with proper SSR handling
3. Created NavbarItem/Custom.tsx to wrap the AuthDropdown component
4. Removed incorrect implementation files

## Prompt Text
✅ Fix Custom Auth Dropdown in Docusaurus v3 (Final)
❌ Problem Summary

You were getting errors like:

Bad navbar item type authDropdown
No NavbarItem component found for type "custom-auth-dropdown"
Element type is invalid

❗ Root Cause

In Docusaurus v3:

Custom navbar item types are NOT allowed

ComponentTypes override no longer exists

Only built-in types or the special type custom is supported

✅ The ONLY Supported Solution

You must use:

type: "custom"


And implement:

src/theme/NavbarItem/Custom.tsx

## Response Text
Successfully implemented the correct Docusaurus v3 solution:

1. Updated docusaurus.config.ts to use type: 'custom' instead of 'custom-auth-dropdown'
2. Created src/components/AuthDropdown.tsx with the authentication dropdown component
3. Created src/theme/NavbarItem/Custom.tsx that wraps the AuthDropdown component
4. Removed all incorrect implementation files

The solution now follows Docusaurus v3 standards where:
- type: 'custom' maps to NavbarItem/Custom.tsx automatically
- The component properly handles server-side rendering to prevent build hanging
- Authentication state is checked only on the client side
