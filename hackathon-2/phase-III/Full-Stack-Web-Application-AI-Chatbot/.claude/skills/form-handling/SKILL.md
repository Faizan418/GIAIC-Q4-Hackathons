---
name: form-handling
description: Implements robust form handling with validation, error display, loading states, server actions, and accessibility features.
---

# Form Handling Skill

This skill creates reliable, user-friendly forms with modern Next.js patterns.

## Best Practices
- Use Server Actions for mutations
- Validate on server (primary) + client (UX)
- Show inline errors
- Loading/disabled states
- Proper labels and ARIA

## Server Action Form Pattern
```tsx
// app/profile/page.tsx
async function updateProfile(formData: FormData) {
  "use server";
  const session = await getSession();
  // validate and update
}

export default function ProfileForm() {
  const [pending, startTransition] = useTransition();

  return (
    <form action={updateProfile}>
      <input name="name" defaultValue={user.name} required />
      <button disabled={pending}>
        {pending ? "Saving..." : "Save"}
      </button>
    </form>
  );
}