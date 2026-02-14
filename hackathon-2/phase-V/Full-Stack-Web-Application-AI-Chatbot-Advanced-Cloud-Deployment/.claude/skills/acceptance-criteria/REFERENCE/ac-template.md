# Acceptance Criteria Template

#### Acceptance Criteria

**Scenario: User enables dark mode**
- Given the user is on any page
- When they click the theme toggle
- Then the entire UI immediately switches to dark colors
- And the toggle shows "light mode" state
- And the preference is saved

**Scenario: Page reload**
- Given the user previously selected dark mode
- When they reload or navigate
- Then the app loads in dark mode

**Scenario: System preference (no manual choice)**
- Given the user has never toggled theme
- When their OS is in dark mode
- Then the app uses dark theme

**Scenario: Error case - localStorage disabled**
- Given localStorage is blocked
- When user toggles theme
- Then theme changes for current session only
- And no error is thrown