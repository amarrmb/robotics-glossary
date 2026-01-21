#!/bin/bash
# ralph-wiggum.sh - "I'm helping!"
# One file. One validation. One commit. Repeat.
#
# Usage: ./scripts/ralph-wiggum.sh <task-file>
# Example: ./scripts/ralph-wiggum.sh hardware/compute/jetson-orin.mdx
#
# Or run in loop mode:
# ./scripts/ralph-wiggum.sh --loop

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
CONTENT_DIR="$PROJECT_ROOT/src/content/docs"
PLAN_FILE="$PROJECT_ROOT/EXECUTION_PLAN.md"
PROGRESS_FILE="$PROJECT_ROOT/.ralph-progress"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_ralph() {
    echo -e "${YELLOW}🍩 Ralph says:${NC} $1"
}

print_phase() {
    echo -e "\n${BLUE}═══════════════════════════════════════════════════════════${NC}"
    echo -e "${BLUE}  Phase: $1${NC}"
    echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}\n"
}

# Get list of files to process (Priority order from EXECUTION_PLAN.md)
get_task_queue() {
    cat << 'EOF'
hardware/compute/jetson-orin.mdx
hardware/compute/jetson-thor.mdx
hardware/compute/dgx-spark.mdx
software/isaac/isaac-ros.mdx
software/simulation/isaac-sim.mdx
software/frameworks/ros2.mdx
concepts/perception/slam.mdx
hardware/sensors/lidar.mdx
hardware/sensors/cameras.mdx
hardware/sensors/imu.mdx
concepts/fundamentals/kinematics.mdx
concepts/fundamentals/degrees-of-freedom.mdx
concepts/control/motion-planning.mdx
concepts/control/pid.mdx
concepts/ai/neural-networks.mdx
concepts/ai/reinforcement-learning.mdx
EOF
}

# Get next unprocessed file
get_next_task() {
    touch "$PROGRESS_FILE"
    while IFS= read -r task; do
        if ! grep -qxF "$task" "$PROGRESS_FILE" 2>/dev/null; then
            echo "$task"
            return 0
        fi
    done < <(get_task_queue)
    echo ""
}

# Mark task as done
mark_done() {
    echo "$1" >> "$PROGRESS_FILE"
}

# Generate the Claude Code prompt for a task
generate_prompt() {
    local task_file="$1"
    local phase="$2"
    local full_path="$CONTENT_DIR/$task_file"

    case "$phase" in
        research)
            cat << EOF
## Task: Research phase for \`$task_file\`

**Date**: $(date +%Y-%m-%d)
**Objective**: Gather current, accurate information to validate/update this glossary entry.

### Instructions:

1. **Read the current file**: \`$full_path\`

2. **Research current state** (use WebSearch and WebFetch):
   - Official NVIDIA documentation (developer.nvidia.com)
   - Official ROS documentation (docs.ros.org)
   - Release notes and changelogs
   - GitHub repos for version numbers

3. **Document findings** in \`$PROJECT_ROOT/research/$task_file.research.md\`:
   - Current version numbers (JetPack, Isaac ROS, ROS distros)
   - Any deprecated features or APIs
   - New features since last update
   - Corrections needed
   - Sources with URLs

4. **Do NOT edit the content file yet** - research only.

### Output expected:
Create research file with findings and sources.
EOF
            ;;
        implement)
            cat << EOF
## Task: Implement phase for \`$task_file\`

**Date**: $(date +%Y-%m-%d)
**Objective**: Update the glossary entry based on research findings.

### Instructions:

1. **Read the research**: \`$PROJECT_ROOT/research/$task_file.research.md\`

2. **Read the current content**: \`$full_path\`

3. **Update the content file**:
   - Add \`last_validated: $(date +%Y-%m-%d)\` to frontmatter
   - Update version numbers to current
   - Fix any inaccuracies found in research
   - Add/update Sources section at bottom with citations
   - Keep content concise - quality over quantity
   - Preserve the existing structure and tone

4. **Citation format**:
\`\`\`markdown
## Sources

- [NVIDIA Isaac ROS Documentation](url) — Version and feature reference
- [ROS 2 Humble Release Notes](url) — Distro status
\`\`\`

### Quality checklist:
- [ ] Frontmatter has last_validated date
- [ ] Version numbers are current (Jan 2026)
- [ ] No broken or outdated links
- [ ] Sources section exists with working URLs
- [ ] Content is accurate per research

### Output expected:
Updated content file with proper citations.
EOF
            ;;
        review)
            cat << EOF
## Task: Review phase for \`$task_file\`

**Date**: $(date +%Y-%m-%d)
**Objective**: Final quality check and plan update.

### Instructions:

1. **Read the updated content**: \`$full_path\`

2. **Verify quality gates**:
   - [ ] \`last_validated\` in frontmatter matches today
   - [ ] All version numbers verified against sources
   - [ ] Sources section present with working links
   - [ ] No marketing fluff - honest, helpful content
   - [ ] Prerequisites and Related Terms links work
   - [ ] Code examples are current and functional

3. **If issues found**: List them and fix.

4. **Update EXECUTION_PLAN.md**:
   - Change status from 🔴 to ✅ for this file
   - Add any notes about what changed

5. **Create summary** in \`$PROJECT_ROOT/research/$task_file.review.md\`:
   - What was updated
   - Key changes made
   - Any remaining concerns

### Output expected:
- Verified content file
- Updated EXECUTION_PLAN.md
- Review summary file
EOF
            ;;
    esac
}

# Main execution for a single task
run_task() {
    local task_file="$1"
    local full_path="$CONTENT_DIR/$task_file"

    if [[ ! -f "$full_path" ]]; then
        echo -e "${RED}Error: File not found: $full_path${NC}"
        exit 1
    fi

    print_ralph "Working on: $task_file"

    # Create research directory if needed
    mkdir -p "$PROJECT_ROOT/research/$(dirname "$task_file")"

    # Git checkpoint before starting
    print_phase "Git Checkpoint (Before)"
    cd "$PROJECT_ROOT"
    git add -A
    if ! git diff --cached --quiet; then
        git commit -m "checkpoint: before updating $task_file

Ralph Wiggum automation - pre-update state"
        echo -e "${GREEN}Created checkpoint commit${NC}"
    else
        echo -e "${YELLOW}No changes to commit (clean state)${NC}"
    fi

    # Phase 1: Research
    print_phase "1/3 RESEARCH"
    echo "Prompt for Claude Code:"
    echo "----------------------------------------"
    generate_prompt "$task_file" "research"
    echo "----------------------------------------"
    echo ""
    read -p "Press Enter after research is complete..."

    # Phase 2: Implement
    print_phase "2/3 IMPLEMENT"
    echo "Prompt for Claude Code:"
    echo "----------------------------------------"
    generate_prompt "$task_file" "implement"
    echo "----------------------------------------"
    echo ""
    read -p "Press Enter after implementation is complete..."

    # Phase 3: Review
    print_phase "3/3 REVIEW"
    echo "Prompt for Claude Code:"
    echo "----------------------------------------"
    generate_prompt "$task_file" "review"
    echo "----------------------------------------"
    echo ""
    read -p "Press Enter after review is complete..."

    # Git commit after completion
    print_phase "Git Checkpoint (After)"
    git add -A
    if ! git diff --cached --quiet; then
        git commit -m "docs: validate and update $task_file

- Validated against Jan 2026 sources
- Updated version numbers and links
- Added sources/citations section"
        echo -e "${GREEN}Created completion commit${NC}"
    else
        echo -e "${YELLOW}No changes to commit${NC}"
    fi

    # Mark as done
    mark_done "$task_file"

    print_ralph "Done with $task_file! 🎉"
}

# Loop mode - process all tasks
run_loop() {
    print_ralph "Loop mode activated! Let's help ALL the files!"

    while true; do
        local next_task=$(get_next_task)

        if [[ -z "$next_task" ]]; then
            print_ralph "All done! No more files to process! 🎉"
            echo ""
            echo "Progress saved in: $PROGRESS_FILE"
            echo "To reset and start over: rm $PROGRESS_FILE"
            exit 0
        fi

        echo ""
        echo -e "${GREEN}Next task: $next_task${NC}"
        read -p "Process this file? [Y/n/skip/quit] " response

        case "$response" in
            n|N|quit|q)
                print_ralph "Bye bye! Progress saved."
                exit 0
                ;;
            skip|s)
                mark_done "$next_task"
                print_ralph "Skipped $next_task"
                continue
                ;;
            *)
                run_task "$next_task"
                ;;
        esac
    done
}

# Show status
show_status() {
    echo -e "${BLUE}Ralph Wiggum Progress Report${NC}"
    echo "=============================="
    echo ""

    local total=$(get_task_queue | wc -l)
    local done=0

    if [[ -f "$PROGRESS_FILE" ]]; then
        done=$(wc -l < "$PROGRESS_FILE")
    fi

    echo "Completed: $done / $total"
    echo ""

    echo "Remaining tasks:"
    while IFS= read -r task; do
        if ! grep -qxF "$task" "$PROGRESS_FILE" 2>/dev/null; then
            echo "  🔴 $task"
        fi
    done < <(get_task_queue)

    echo ""
    echo "Completed tasks:"
    if [[ -f "$PROGRESS_FILE" ]]; then
        while IFS= read -r task; do
            echo "  ✅ $task"
        done < "$PROGRESS_FILE"
    else
        echo "  (none yet)"
    fi
}

# Main
case "${1:-}" in
    --loop|-l)
        run_loop
        ;;
    --status|-s)
        show_status
        ;;
    --reset)
        rm -f "$PROGRESS_FILE"
        print_ralph "Progress reset! Starting fresh."
        ;;
    --help|-h|"")
        echo "Ralph Wiggum - Glossary Update Automation"
        echo ""
        echo "Usage:"
        echo "  $0 <file>        Process a specific file"
        echo "  $0 --loop        Process all files in queue"
        echo "  $0 --status      Show progress"
        echo "  $0 --reset       Reset progress tracker"
        echo ""
        echo "Example:"
        echo "  $0 hardware/compute/jetson-orin.mdx"
        echo "  $0 --loop"
        ;;
    *)
        run_task "$1"
        ;;
esac
