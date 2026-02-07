# Design Token Sync Best Practices

1. **Source of Truth**: Always Figma (or your design tool)
2. **Export Process**:
   - Use Tokens Studio plugin (recommended)
   - Export as multi-file JSON
   - Commit raw tokens to repo
3. **Build Pipeline**:
   - Generate multiple outputs: CSS variables, TypeScript, Tailwind config, SCSS
   - Version tokens with semantic versioning
4. **Theme Support**:
   - Separate light/dark themes
   - Allow brand overrides
5. **Validation**:
   - JSON schema validation
   - Lint rules for token naming