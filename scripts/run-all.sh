#!/bin/bash

PROJECT="robotics-glossary"
PROJECT_DIR=~/Source/$PROJECT
TODAY=$(date +%Y-%m-%d)

cd "$PROJECT_DIR" || {
    echo "Cannot find $PROJECT_DIR"
    exit 1
}

# Task list
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

PROGRESS_FILE="$PROJECT_DIR/.ralph-progress"

echo "=== Starting Ralph Wiggum Loop for $PROJECT ==="
echo "=== Working in: $PROJECT_DIR ==="
echo "=== Date: $TODAY ==="

# Ensure directories exist
mkdir -p "$PROJECT_DIR/research/hardware/compute"
mkdir -p "$PROJECT_DIR/research/hardware/sensors"
mkdir -p "$PROJECT_DIR/research/software/isaac"
mkdir -p "$PROJECT_DIR/research/software/simulation"
mkdir -p "$PROJECT_DIR/research/software/frameworks"
mkdir -p "$PROJECT_DIR/research/concepts/perception"
mkdir -p "$PROJECT_DIR/research/concepts/fundamentals"
mkdir -p "$PROJECT_DIR/research/concepts/control"
mkdir -p "$PROJECT_DIR/research/concepts/ai"

touch "$PROGRESS_FILE"

# Initialize git if needed
if [ ! -d ".git" ]; then
    git init -b main
    git add -A && git commit -m "Initial commit for $PROJECT"
fi

# Create starting tag
git add -A && git commit -m "Pre-validation state" --allow-empty 2>/dev/null
git tag "ralph-start-$(date +%Y%m%d-%H%M%S)" 2>/dev/null

is_done() {
    grep -qxF "$1" "$PROGRESS_FILE" 2>/dev/null
}

mark_done() {
    echo "$1" >> "$PROGRESS_FILE"
}

TASK_NUM=0
TOTAL=${#TASKS[@]}

for task in "${TASKS[@]}"; do
    TASK_NUM=$((TASK_NUM + 1))

    if is_done "$task"; then
        echo "✅ SKIP (done): $task"
        continue
    fi

    echo ""
    echo "============================================"
    echo "=== $PROJECT - Task $TASK_NUM of $TOTAL ==="
    echo "=== File: $task ==="
    echo "============================================"

    TASK_NAME=$(basename "$task" .mdx)
    RESEARCH_FILE="research/${task%.mdx}.research.md"

    # Tag before each task for rollback
    git tag "task-$TASK_NUM-start" 2>/dev/null

    echo ">>> Phase 1: RESEARCH"
    claude --dangerously-skip-permissions -p "You are working on the $PROJECT project in $PROJECT_DIR.

Your task: Research and validate the glossary entry at src/content/docs/$task

Instructions:
1. Read the current file: src/content/docs/$task
2. Use WebSearch to find current official information:
   - NVIDIA official documentation (developer.nvidia.com)
   - ROS documentation (docs.ros.org)
   - Current version numbers as of January 2026
   - Any deprecations or major changes
3. Create research notes at: $RESEARCH_FILE
   - Document current version numbers found
   - Note any corrections needed to our content
   - Include source URLs for every fact
   - Keep it concise - bullet points preferred

Do NOT modify the content file yet. Research only.

After research:
git add -A && git commit -m '[$PROJECT] $TASK_NAME - Research complete'"

    echo ">>> Phase 2: IMPLEMENT"
    claude --dangerously-skip-permissions -p "You are working on the $PROJECT project in $PROJECT_DIR.

Your task: Update the glossary entry based on research findings.

Instructions:
1. Read your research notes: $RESEARCH_FILE
2. Update src/content/docs/$task:
   - Add to frontmatter: last_validated: $TODAY
   - Update any stale version numbers to current (Jan 2026)
   - Fix any inaccuracies found in research
   - Add/update Sources section at bottom:
     ## Sources
     - [Source Name](url) — What this source provided
3. Keep content concise - quality over quantity
4. Do not add fluff or marketing language

After implementation:
git add -A && git commit -m '[$PROJECT] $TASK_NAME - Content updated and validated'"

    echo ">>> Phase 3: REVIEW"
    claude --dangerously-skip-permissions -p "You are working on the $PROJECT project in $PROJECT_DIR.

Your task: Final review and update tracking.

Instructions:
1. Re-read the updated file: src/content/docs/$task
2. Verify:
   - last_validated: $TODAY is in frontmatter
   - Version numbers match research findings
   - Sources section exists with working URLs
   - Internal links are not broken
   - Content is accurate and helpful
3. Fix any issues found
4. Update EXECUTION_PLAN.md: find the row for \`$task\` and change 🔴 to ✅

After review:
git add -A && git commit -m '[$PROJECT] $TASK_NAME - Reviewed and verified'"

    # Tag after task completes
    git tag "task-$TASK_NUM-complete" 2>/dev/null

    # Mark as done in progress file
    mark_done "$task"

    echo "=== Task $TASK_NUM ($task) complete ==="
    sleep 2
done

echo ""
echo "============================================"
echo "=== $PROJECT - FINAL AUDIT ==="
echo "============================================"

claude --dangerously-skip-permissions -p "You are working on the $PROJECT project in $PROJECT_DIR.

Perform a complete audit of the validation work:

1. Check EXECUTION_PLAN.md - are all files marked ✅?
2. Review .ralph-progress - does it list all 16 files?
3. Spot-check 2-3 random files to verify:
   - last_validated date is present
   - Sources section exists
4. Generate a summary and save to VALIDATION_AUDIT.md:
   - Files validated
   - Any files that need attention
   - Overall status

Final commit: git add -A && git commit -m '[$PROJECT] Validation audit complete - $TODAY'"

git tag "ralph-complete-$(date +%Y%m%d-%H%M%S)" 2>/dev/null

echo ""
echo "============================================"
echo "=== $PROJECT Ralph Wiggum Loop Complete ==="
echo "============================================"
echo ""
echo "Git history:"
git log --oneline -20
echo ""
echo "Tags created:"
git tag -l "task-*" | tail -10
echo ""
echo "Progress file:"
cat "$PROGRESS_FILE"
echo ""
echo "Rollback commands:"
echo "  git reset --hard task-N-start     # Rollback to before task N"
echo "  git reset --hard ralph-start-*    # Rollback to beginning"
echo "  rm .ralph-progress                # Reset progress tracking"
