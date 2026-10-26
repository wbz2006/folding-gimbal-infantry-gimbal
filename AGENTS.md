# Repository Guidance

- Prefer direct `if-else` or `switch` control flow when a ternary hides meaningful state or enum choices.
- Keep simple fallback expressions as ternaries when they remain easier to scan.
- Do not add helper functions or abstractions solely to replace a small conditional; add them only when they clarify a reusable operation or remove meaningful duplication.
