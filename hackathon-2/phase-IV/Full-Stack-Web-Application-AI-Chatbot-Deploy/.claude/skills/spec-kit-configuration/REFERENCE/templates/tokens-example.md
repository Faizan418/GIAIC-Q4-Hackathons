# Design Tokens Example Structure

## Core Tokens (JSON Format)
```json
{
  "colors": {
    "primary": "#0066FF",
    "secondary": "#FF6600",
    "success": "#00CC66",
    "danger": "#FF3366",
    "background": "#FFFFFF",
    "surface": "#F8F9FA",
    "text": "#1A1A1A"
  },
  "spacing": {
    "xs": "4px",
    "sm": "8px",
    "md": "16px",
    "lg": "24px",
    "xl": "32px"
  },
  "radii": {
    "sm": "4px",
    "md": "8px",
    "lg": "12px",
    "full": "9999px"
  },
  "typography": {
    "fontFamily": "Inter, system-ui, sans-serif",
    "fontSize": {
      "base": "16px",
      "sm": "14px",
      "lg": "18px",
      "xl": "24px"
    }
  }
}
Semantic Tokens
JSON{
  "colors": {
    "brand": "{colors.primary}",
    "accent": "{colors.secondary}",
    "on-brand": "#FFFFFF",
    "border": "#E0E0E0"
  }
}
Component Specs Example (Button)
JSON{
  "button": {
    "variants": ["primary", "secondary", "ghost", "outline"],
    "sizes": ["sm", "md", "lg"],
    "states": ["default", "hover", "active", "disabled"],
    "defaultProps": {
      "variant": "primary",
      "size": "md"
    }
  }
}