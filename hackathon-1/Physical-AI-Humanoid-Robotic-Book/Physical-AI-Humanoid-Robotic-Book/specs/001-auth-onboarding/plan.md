# Implementation Plan: Auth & Onboarding System

**Branch**: `001-auth-onboarding` | **Date**: 2025-12-18 | **Spec**: [specs/001-auth-onboarding/spec.md](specs/001-auth-onboarding/spec.md)
**Input**: Feature specification from `/specs/001-auth-onboarding/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of a comprehensive authentication and onboarding system using Better Auth for the Physical AI & Humanoid Robotics textbook platform. The system will handle user registration, secure authentication, background profiling, content personalization, and animated UI experiences. The implementation follows Next.js App Router with TypeScript, with proper CI/CD governance and constitution compliance.

## Technical Context

**Language/Version**: TypeScript with Next.js App Router
**Primary Dependencies**: Better Auth, Next.js, CSS for animations
**Storage**: Application database for profile data (separate from auth)
**Testing**: Unit tests, integration tests, workflow validation
**Target Platform**: Web application (Next.js)
**Project Type**: Frontend authentication and personalization system
**Performance Goals**: <100ms UI animation delays, under 5-minute onboarding flow
**Constraints**: No inline styles, motion under 600ms, keyboard accessibility, profile completion required for full access
**Scale/Scope**: Support for thousands of users with personalized content delivery, multiple learning tracks (SOFTWARE_ONLY, HARDWARE_ONLY, FULL_ROBOTICS)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Based on the constitution file, the following gates apply:
- ✅ **Authentication Stack Governance**: Using Next.js (App Router) + TypeScript with Better Auth, cookie-based sessions with profile storage in application database (compliant)
- ✅ **Signup & Signin Flow Requirements**: Following constitution-defined flows (email+password → Better Auth → onboarding → profile persistence → dashboard) (compliant)
- ✅ **User Background Profiling**: Collecting mandatory software/hardware background and learning intent as specified (compliant)
- ✅ **Better Auth Configuration Standards**: Using email/password with cookie sessions as required (compliant)
- ✅ **Animated UI Constitution**: Following CSS-only animations with hardware-accelerated transforms, no inline styles, motion under 600ms (compliant)
- ✅ **Enforcement Rules**: Reusing shared CSS, no inline styles, keyboard accessibility (compliant)
- ✅ **Personalization Contract**: Applying BEGINNER/HARDWARE_ONLY/FULL_ROBOTICS rules as specified (compliant)
- ✅ **GitHub Actions Workflow Requirements**: Implementing ci.yml, auth-check.yml, and deploy.yml workflows (compliant)

*Post-design constitution check: All requirements satisfied with proper authentication flows, personalization logic, and workflow enforcement.*

## Project Structure

### Documentation (this feature)

```text
specs/001-auth-onboarding/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (Next.js application)

```text
app/
├── (auth)/
│   ├── layout.tsx
│   ├── signin/
│   │   └── page.tsx
│   ├── signup/
│   │   └── page.tsx
│   └── middleware.ts
├── onboarding/
│   ├── layout.tsx
│   ├── page.tsx
│   └── components/
│       ├── SoftwareBackgroundForm.tsx
│       ├── HardwareBackgroundForm.tsx
│       └── LearningTrackForm.tsx
├── lib/
│   └── auth.ts
├── styles/
│   └── auth.css
├── middleware.ts
└── .github/
    └── workflows/
        ├── ci.yml
        ├── auth-check.yml
        └── deploy.yml
```

**Structure Decision**: Next.js App Router structure following the directory convention specified in the feature requirements. Authentication flows are isolated in the `(auth)` group to maintain proper routing and access control. The onboarding flow is a separate route that requires authentication but validates profile completeness.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
