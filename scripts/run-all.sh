#!/bin/bash
# run-all.sh - Process all files in sequence
# Usage: ./scripts/run-all.sh
#
# This script loops through all task files and runs Claude Code on each.
# Progress is tracked in .ralph-progress

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
PROGRESS_FILE="$PROJECT_ROOT/.ralph-progress"

# Task queue (priority order)
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

# Ensure research directories exist
mkdir -p "$PROJECT_ROOT/research/hardware/compute"
mkdir -p "$PROJECT_ROOT/research/hardware/sensors"
mkdir -p "$PROJECT_ROOT/research/software/isaac"
mkdir -p "$PROJECT_ROOT/research/software/simulation"
mkdir -p "$PROJECT_ROOT/research/software/frameworks"
mkdir -p "$PROJECT_ROOT/research/concepts/perception"
mkdir -p "$PROJECT_ROOT/research/concepts/fundamentals"
mkdir -p "$PROJECT_ROOT/research/concepts/control"
mkdir -p "$PROJECT_ROOT/research/concepts/ai"

# Initialize progress file
touch "$PROGRESS_FILE"

is_done() {
    grep -qxF "$1" "$PROGRESS_FILE" 2>/dev/null
}

mark_done() {
    echo "$1" >> "$PROGRESS_FILE"
}

echo "═══════════════════════════════════════════════════════════"
echo "  Ralph Wiggum Batch Runner"
echo "  'I'm helping!'"
echo "═══════════════════════════════════════════════════════════"
echo ""

# Initial git commit
cd "$PROJECT_ROOT"
git add -A
if ! git diff --cached --quiet 2>/dev/null; then
    git commit -m "checkpoint: starting batch validation run

Date: $(date +%Y-%m-%d)
Files to process: ${#TASKS[@]}"
fi

for task in "${TASKS[@]}"; do
    if is_done "$task"; then
        echo "✅ SKIP (already done): $task"
        continue
    fi

    echo ""
    echo "═══════════════════════════════════════════════════════════"
    echo "  Processing: $task"
    echo "═══════════════════════════════════════════════════════════"
    echo ""

    # Generate and display the prompt
    prompt=$("$SCRIPT_DIR/claude-prompt.sh" "$task")

    echo "Run this in Claude Code:"
    echo "────────────────────────────────────────────────────────────"
    echo ""
    echo "$prompt"
    echo ""
    echo "────────────────────────────────────────────────────────────"
    echo ""

    # Or pipe directly to claude if available
    # echo "$prompt" | claude

    read -p "Press Enter when Claude Code has finished this task (or 's' to skip, 'q' to quit): " response

    case "$response" in
        s|S|skip)
            echo "Skipped $task"
            ;;
        q|Q|quit)
            echo "Exiting. Progress saved."
            exit 0
            ;;
        *)
            mark_done "$task"
            echo "✅ Marked as done: $task"
            ;;
    esac
done

echo ""
echo "═══════════════════════════════════════════════════════════"
echo "  All tasks completed!"
echo "═══════════════════════════════════════════════════════════"
echo ""
echo "To reset progress: rm $PROGRESS_FILE"
