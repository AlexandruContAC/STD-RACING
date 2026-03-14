# Agent Efficiency & Token Optimization Rules

## 1. Context Minimization
- **Strict Scope:** Only read files explicitly mentioned or directly related to the current task. Do not perform "Full Repository Scans" unless a dependency is missing.
- **Diffs Only:** When proposing changes, provide code blocks for the specific lines being changed rather than reprinting the entire file.
- **State Management:** Before performing an action, check the terminal history or previous Artifacts to ensure you aren't repeating a failed command.

## 2. Tool Usage Constraints
- **Terminal:** Run commands in batches (e.g., `npm install && npm run build`) rather than individual calls to save round-trip tokens.
- **Browser Agent:** Do NOT use the browser for documentation lookup if the relevant `README.md` or `docs/` folder exists locally.
- **Artifacts:** Only generate a new "Plan Artifact" if the task involves more than 3 files. For single-file edits, use direct chat response.

## 3. Communication Style
- **Conciseness:** Be brief. Avoid conversational filler (e.g., "Certainly, I can help with that"). 
- **Dry Execution:** If a command is successful, simply report "Success" and move to the next step.
- **Stop Loss:** If a task fails 3 times in a row, STOP and ask the user for clarification instead of attempting a 4th automated fix.

## 4. Model Selection (Dynamic)
- For routine documentation updates or simple CSS tweaks, prompt the user to switch the session to **Gemini 3 Flash** if it is currently set to Pro/Ultra.
