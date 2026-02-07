Markdown# Essential Root-Level Files Guide

## 1. package.json (Root)
- Must have "private": true
- Define common scripts
- List shared devDependencies

## 2. tsconfig.json (Root Base)
- Enable composite: true for project references
- Define path aliases
- Strict mode enabled

## 3. eslint.config.js (Flat Config)
```js
module.exports = [
  ...require('@eslint/js').configs.recommended,
  {
    rules: {
      "no-console": "warn",
      "import/order": "error"
    }
  }
];
4. .prettierrc
JSON{
  "semi": true,
  "trailingComma": "es5",
  "singleQuote": true,
  "printWidth": 80,
  "tabWidth": 2
}
5. .gitign