#!/bin/bash
# claude-prompt.sh - Generate prompt for Claude Code
# Usage: ./scripts/claude-prompt.sh <file> | claude
# Or: claude "$(./scripts/claude-prompt.sh hardware/compute/jetson-orin.mdx)"

TASK_FILE="${1:-hardware/compute/jetson-orin.mdx}"
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TODAY=$(date +%Y-%m-%d)

cat << EOF
You are updating the robotics-glossary project. Execute these 3 phases for: $TASK_FILE

## Context
- Project: NVIDIA + ROS2 robotics glossary for thought leadership
- Philosophy: Quality > quantity. Not a docs clone. Helpful summaries with proper citations.
- Validation date: $TODAY

---

## PHASE 1: RESEARCH

1. Read the current file: src/content/docs/$TASK_FILE
2. Use WebSearch to find current official information:
   - Search for current version numbers (JetPack, Isaac ROS, ROS 2 distros)
   - Search for any recent changes or deprecations
   - Search official NVIDIA robotics documentation
   - Search ROS 2 documentation if relevant
3. Create research notes in: research/${TASK_FILE%.mdx}.research.md
   - List current versions found
   - Note any corrections needed
   - Include source URLs for every fact

---

## PHASE 2: IMPLEMENT

1. Read your research notes
2. Update src/content/docs/$TASK_FILE:

   a. Add to frontmatter:
   \`\`\`yaml
   last_validated: $TODAY
   \`\`\`

   b. Update any stale version numbers to current

   c. Add/update Sources section at the end:
   \`\`\`markdown
   ## Sources

   - [Source Name](url) — What this source provided
   \`\`\`

3. Keep content concise and helpful. Don't pad with fluff.

---

## PHASE 3: REVIEW

1. Re-read the updated file
2. Verify:
   - [ ] last_validated is $TODAY
   - [ ] Version numbers match research
   - [ ] Sources section has working URLs
   - [ ] No broken internal links
   - [ ] Content is accurate and helpful

3. Update EXECUTION_PLAN.md: change this file's status from 🔴 to ✅

4. Create git commits:
   \`\`\`bash
   git add -A && git commit -m "docs: validate and update $TASK_FILE

   - Validated against $TODAY sources
   - Updated version numbers
   - Added citations"
   \`\`\`

---

## OUTPUT SUMMARY

After completing all phases, tell me:
1. What version numbers were updated
2. What corrections were made
3. What sources were added
4. Any concerns or items needing human review

Begin with Phase 1: Research.
EOF
