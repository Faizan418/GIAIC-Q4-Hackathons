# Dark Mode Toggle

## Overview
Allow users to switch between light and dark themes across the entire application.

## Goals
- Improve accessibility and user comfort
- Match industry standard

## Non-Goals
- Custom theme colors
- Per-page theme overrides

## Technical Details
Use existing spec-kit tokens with a new "theme" context. Toggle persists in localStorage and user preferences.