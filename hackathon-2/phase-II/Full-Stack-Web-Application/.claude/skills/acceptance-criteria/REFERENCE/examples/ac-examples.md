**Scenario: Invalid login**
- Given the user is on login page
- When they enter wrong password
- Then error message "Invalid credentials" appears
- And password field is cleared
- And email field retains value
- And login attempt is rate-limited after 5 failures