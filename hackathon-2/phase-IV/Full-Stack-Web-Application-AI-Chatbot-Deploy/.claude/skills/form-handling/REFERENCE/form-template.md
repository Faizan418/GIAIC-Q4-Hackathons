# Form Handling Examples

## Zod Validation + Server Action
```tsx
import { z } from 'zod';

const schema = z.object({
  email: z.string().email(),
  password: z.string().min(12)
});

async function login(formData: FormData) {
  "use server";
  const data = Object.fromEntries(formData);
  const parsed = schema.safeParse(data);
  if (!parsed.success) {
    return { errors: parsed.error.flatten() };
  }
  // proceed
}
Client-Side Validation
tsx'use client';

import { useFormState } from 'react-dom';

const [state, formAction] = useFormState(updateProfile, null);

<form action={formAction}>
  {state?.errors?.name && <p className="text-red-500">{state.errors.name}</p>}
  <input name="name" />
</form>
```