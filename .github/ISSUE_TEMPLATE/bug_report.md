---
name: Bug Report
about: Create a report to help us improve
title: "[BUG] Short description of the issue"
labels: bug
assignees: ''
---

## 🔴 Description
A clear and concise description of what the bug is.

## 👣 Steps to Reproduce
1. Go to '...'
2. Call function '...' with arguments '...'
3. See error

## 📉 Expected vs Actual Behavior
* **Expected:** The rocket should detect apogee.
* **Actual:** The state machine remained in `ASCENT` state.

## 🔍 Root Cause Analysis (Preliminary)
* Potential cause: ...

## 💻 Environment
* **Hardware:** (e.g., ESP32, T3.6)
* **PlatformIO Env:** (e.g., `native`, `rocket_v1`)
* **Commit SHA:** ...

## 🧪 Regression Test
* [ ] Create a failing test case that reproduces this bug.
* [ ] Verify the test passes after the fix.
