# MeshCom Firmware - Development Guidelines

## Project Context
This is an open-source project (MeshCom Firmware). We contribute via PRs against the **upstream DEV branch** of the icssw-org repository.

## PR Workflow

### 1. Sync Upstream First
Before any coding, always sync/rebase against the latest upstream DEV branch to incorporate all upstream changes.

### 2. Minimal Changes Only
We **cherry-pick the absolute minimum** of code changes. We do NOT rewrite or refactor large parts of the project. Every change must be targeted and justified.

### 3. PR Description (German, Detailed)
Every PR **must** include a detailed description written in **German**:
- Describe exactly which code was changed (files, functions, logic)
- Explain **why** each change was made (motivation, bug fix rationale, improvement reason)
- This description must be prepared **before** submitting the PR

### 4. PR Target
All PRs target the **DEV branch** of the upstream repository (not main).


