#!/bin/bash
# auto-run.sh - Fully automated, pipes directly to claude
# Usage: ./scripts/auto-run.sh <file>
# Example: ./scripts/auto-run.sh hardware/compute/jetson-orin.mdx
#
# Or run all:
# ./scripts/auto-run.sh --all

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
PROGRESS_FILE="$PROJECT_ROOT/.ralph-progress"
TODAY=$(date +%Y-%m-%d)

TASKS=(
    "hardware/compute/jetson-orin.mdx"
    "hardware/compute/jetson-thor.mdx"
    "hardware/compute/dgx-spark.mdx"
    "software/isaac/isaac-ros.mdx"
    "software/simulation/isaac-sim.mdx"
    "software/frameworks/ros2.mdx"
    "concepts/perception/slam.mdx"
    "hardware/sensors/lidar.mdx"
    "hardware/sensors/cameras.mdx"
    "hardware/sensors/imu.mdx"
    "concepts/fundamentals/kinematics.mdx"
    "concepts/fundamentals/degrees-of-freedom.mdx"
    "concepts/control/motion-planning.mdx"
    "concepts/control/pid.mdx"
    "concepts/ai/neural-networks.mdx"
    "concepts/ai/reinforcement-learning.mdx"
)

generate_prompt() {
    local task="$1"
    cat << EOF
Execute these 3 phases for robotics-glossary file: $task

Project context: NVIDIA + ROS2 robotics glossary. Quality > quantity. Proper citations required.

## PHASE 1: RESEARCH
1. Read: src/content/docs/$task
2. WebSearch for current official info (NVIDIA docs, ROS docs, version numbers)
3. Save findings to: research/${task%.mdx}.research.md (include source URLs)

## PHASE 2: IMPLEMENT
1. Update src/content/docs/$task:
   - Add \`last_validated: $TODAY\` to frontmatter
   - Fix any stale version numbers
   - Add Sources section at bottom with citations
2. Keep content concise and accurate

## PHASE 3: REVIEW & COMMIT
1. Verify the update is complete and accurate
2. In EXECUTION_PLAN.md, find the row for this file and change 🔴 to ✅
3. Run: git add -A && git commit -m "docs: validate $task - $TODAY"

When done, output a brief summary of changes made.
EOF
}

is_done() {
    grep -qxF "$1" "$PROGRESS_FILE" 2>/dev/null
}

mark_done() {
    echo "$1" >> "$PROGRESS_FILE"
}

get_next() {
    touch "$PROGRESS_FILE"
    for task in "${TASKS[@]}"; do
        if ! is_done "$task"; then
            echo "$task"
            return
        fi
    done
}

run_one() {
    local task="$1"
    echo "══════════════════════════════════════════════════"
    echo "Processing: $task"
    echo "══════════════════════════════════════════════════"

    prompt=$(generate_prompt "$task")

    # Pipe to claude
    echo "$prompt" | claude --print

    mark_done "$task"
    echo "✅ Done: $task"
}

case "${1:-}" in
    --all)
        while true; do
            next=$(get_next)
            if [[ -z "$next" ]]; then
                echo "All tasks complete!"
                exit 0
            fi
            run_one "$next"
        done
        ;;
    --next)
        next=$(get_next)
        if [[ -z "$next" ]]; then
            echo "All tasks complete!"
            exit 0
        fi
        run_one "$next"
        ;;
    --status)
        echo "Progress: $(wc -l < "$PROGRESS_FILE" 2>/dev/null || echo 0) / ${#TASKS[@]}"
        echo ""
        echo "Remaining:"
        for task in "${TASKS[@]}"; do
            if ! is_done "$task"; then
                echo "  🔴 $task"
            fi
        done
        ;;
    --prompt)
        # Just output prompt for next task (for manual use)
        next=$(get_next)
        if [[ -n "$next" ]]; then
            generate_prompt "$next"
        fi
        ;;
    "")
        echo "Usage:"
        echo "  $0 <file>     Run one specific file"
        echo "  $0 --next     Run next unprocessed file"
        echo "  $0 --all      Run all remaining files"
        echo "  $0 --status   Show progress"
        echo "  $0 --prompt   Output prompt for next file (manual use)"
        ;;
    *)
        run_one "$1"
        ;;
esac
