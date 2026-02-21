```skill
---
name: python
description: Run Python scripts using miniconda environment
argument-hint: [env-name] <script-or-command>
---

# Python with Miniconda

When running Python commands or scripts, always activate the correct miniconda environment first.

## Conda Location

Miniconda is installed at `~/miniconda3` but `conda` is **not in PATH** in terminal sessions.

## How to Run Python

**Option 1: Direct path (preferred for simple commands)**
```bash
~/miniconda3/envs/<env>/bin/python3 <script.py>
```

**Option 2: Activate environment (for interactive or multi-command sessions)**
```bash
eval "$(~/miniconda3/bin/conda shell.bash hook)" && conda activate <env> && python3 <script.py>
```

**Option 3: pip install**
```bash
~/miniconda3/envs/<env>/bin/pip install <package>
```

## Default Environment

If no environment is specified in `$ARGUMENTS`, use `px4-mcp` as the default.

Parse `$ARGUMENTS`:
- If first word matches an env name (`px4-mcp`, `px4`, `dexvla`), use that env and pass the rest as the command
- Otherwise, use `px4-mcp` and pass all arguments as the command

## Examples

```bash
# Run a Python script with px4-mcp env (default)
~/miniconda3/envs/px4-mcp/bin/python3 Tools/mcp/px4_mcp_server.py

# Run with specific env
~/miniconda3/envs/dexvla/bin/python3 train.py

# Install a package
~/miniconda3/envs/px4-mcp/bin/pip install pyulog

# Activate for interactive use
eval "$(~/miniconda3/bin/conda shell.bash hook)" && conda activate px4-mcp
```
```
