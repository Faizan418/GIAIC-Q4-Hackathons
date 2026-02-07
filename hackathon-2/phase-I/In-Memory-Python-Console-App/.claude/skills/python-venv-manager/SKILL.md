---
name: python-venv-manager
description: Use this agent when managing Python virtual environments for a project. Create, activate, and verify isolated Python environments.
license: MIT
version: 1.0.0
---

# Python Venv Manager Skill

Python Virtual Environment Specialist with expertise in package management, virtual environment creation, and dependency isolation.

## Purpose

Ensure a clean, reproducible Python development environment through proper virtual environment setup and management.

## When to Use

- Starting a new project: "Set up a virtual environment"
- Installing dependencies: "Install dependencies correctly"
- Debugging ModuleNotFoundError: "Packages can't be found"
- Configuring IDE: "Configure VS Code for venv"

## Core Responsibilities

1. **Create Virtual Environments**: Initialize isolated Python environments using `venv`
2. **Activate Environments**: Configure shell to use venv Python interpreter
3. **Manage pip**: Ensure installations occur within venv
4. **Verify Configuration**: Confirm proper setup via validation commands

## Environment Creation

```bash
# Primary method (Python 3.3+)
python -m venv .venv --with-pip

# Default path: .venv in project root
# Handle existing environments gracefully
```

## Environment Activation

| Platform | Command |
|----------|---------|
| Windows | `.venv\Scripts\activate.bat` |
| PowerShell | `.venv\Scripts\Activate.ps1` |
| Linux/macOS | `source .venv/bin/activate` |

### Programmatic Activation
```python
import os
os.environ['VIRTUAL_ENV'] = '/path/to/.venv'
os.environ['PATH'] = '/path/to/.venv/bin:' + os.environ['PATH']
```

## pip Management

- Verify pip is from venv: `which pip` / `Get-Command pip`
- Use `python -m pip install <package>` for reliability
- Create requirements file: `pip freeze > requirements.txt`

## Validation Steps

After any setup operation:
1. `python --version` points to venv Python
2. `pip --version` shows pip from venv path
3. Installed packages are importable

## Error Handling

| Error | Action |
|-------|--------|
| Python not found | Check PATH, offer to find installations |
| venv creation fails | Try --clear flag, clean up partial directory |
| Activation fails | Provide manual activation commands |
| pip install fails | Verify connectivity, check permissions |

## Best Practices

- **Isolation**: Always install in venv, never globally
- **Reproducibility**: Document Python version required
- **Cleanup**: Offer to backup before recreating
- **Cross-platform**: Scripts work on Windows, Linux, macOS
- **IDE Integration**: Advise on configuring VS Code/PyCharm

## Workflow Pattern

```
1. Check if .venv exists
2. If not, create with python -m venv .venv --with-pip
3. Activate (or provide activation commands)
4. Verify environment is active
5. Perform requested task
6. Report status and manual steps needed
```

## Output Expectations

- Clear, step-by-step output of operations
- Success/failure for each operation
- Manual commands user needs to run
- Next steps for project setup
