---
name: SimpleCLI semicolon parsing behavior
description: SimpleCLI parser treats ;; as line delimiter but single ; stays in word tokens — key for script-set command design
created: 2026-08-10T11:03:48.519Z
metadata:
  node_type: memory
  generator: opencode-claude-memory
  type: reference
  originSessionId: ses_014a9aeebffeZN6UkLTY994Aiy
---

The vendored SimpleCLI library (`components/SimpleCLI/SimpleCLI/src/c/parser.c:211`) treats `;;` (double semicolon) as a line delimiter in `parse_lines()`, splitting input into separate command lines. A **single** `;` is kept as part of word tokens — `parse_words()` only splits on spaces (respecting quotes). This means:
- `addBoundlessCmd` gives each space-separated token as a separate arg, with `;` characters preserved inside tokens.
- Joining args with spaces and splitting on `;` in a handler correctly separates commands (used by `script-set`).
- `;;` in a script file fed to `handleCommand` would act as an in-line command separator.
- `;` cannot appear in config values passed through `script-set` (use FTP for scripts with `;` in values).
