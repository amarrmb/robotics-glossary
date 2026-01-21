#!/bin/bash

PROJECT="robotics-glossary"
PROJECT_DIR=~/Source/$PROJECT
TODAY=$(date +%Y-%m-%d)

cd "$PROJECT_DIR" || {
    echo "Cannot find $PROJECT_DIR"
    exit 1
}

PROGRESS_FILE="$PROJECT_DIR/.ralph-progress"

# ============================================
# PHASE 0: VALIDATE EXISTING CONTENT
# ============================================
VALIDATE_TASKS=(
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

# ============================================
# PHASE 1: CREATE NEW CONTENT (Zero to Hero Path)
# ============================================
# Format: "path/file.mdx|Title|Description"
CREATE_TASKS=(
    "concepts/fundamentals/transforms.mdx|Transforms|Coordinate transformations - the foundation of robotics math"
    "concepts/fundamentals/coordinate-frames.mdx|Coordinate Frames|Reference frames and how robots understand position in space"
    "software/ros2/tf2.mdx|TF2|ROS 2 transform library - managing coordinate frames"
    "software/ros2/nav2.mdx|Nav2|ROS 2 Navigation Stack - autonomous mobile robot navigation"
    "software/ros2/moveit2.mdx|MoveIt 2|Motion planning framework for robot arms"
    "software/ros2/urdf-xacro.mdx|URDF & Xacro|Robot description formats for ROS"
    "software/nvidia/cumotion.mdx|cuMotion|NVIDIA GPU-accelerated motion planning"
    "software/nvidia/nvblox.mdx|nvblox|NVIDIA real-time 3D reconstruction"
    "software/nvidia/tensorrt.mdx|TensorRT|NVIDIA deep learning inference optimizer"
    "software/nvidia/isaac-lab.mdx|Isaac Lab|NVIDIA simulation for robot learning"
    "concepts/perception/visual-odometry.mdx|Visual Odometry|Estimating motion from camera images"
    "concepts/perception/sensor-fusion.mdx|Sensor Fusion|Combining multiple sensor inputs"
    "hardware/sensors/depth-cameras.mdx|Depth Cameras|Stereo, ToF, and structured light cameras"
)

echo "=== Starting Ralph Wiggum Loop for $PROJECT ==="
echo "=== Working in: $PROJECT_DIR ==="
echo "=== Date: $TODAY ==="

# Ensure directories exist
mkdir -p "$PROJECT_DIR/research/hardware/compute"
mkdir -p "$PROJECT_DIR/research/hardware/sensors"
mkdir -p "$PROJECT_DIR/research/software/isaac"
mkdir -p "$PROJECT_DIR/research/software/nvidia"
mkdir -p "$PROJECT_DIR/research/software/simulation"
mkdir -p "$PROJECT_DIR/research/software/frameworks"
mkdir -p "$PROJECT_DIR/research/software/ros2"
mkdir -p "$PROJECT_DIR/research/concepts/perception"
mkdir -p "$PROJECT_DIR/research/concepts/fundamentals"
mkdir -p "$PROJECT_DIR/research/concepts/control"
mkdir -p "$PROJECT_DIR/research/concepts/ai"
mkdir -p "$PROJECT_DIR/src/content/docs/software/ros2"
mkdir -p "$PROJECT_DIR/src/content/docs/software/nvidia"

touch "$PROGRESS_FILE"

# ============================================
# RATE LIMIT HANDLING
# ============================================
MAX_RETRIES=3
RETRY_DELAY=60  # seconds
BETWEEN_TASKS_DELAY=5  # seconds between tasks
BETWEEN_PHASES_DELAY=2  # seconds between phases

run_claude() {
    local prompt="$1"
    local attempt=1

    while [ $attempt -le $MAX_RETRIES ]; do
        echo "  [Attempt $attempt/$MAX_RETRIES]"

        # Run claude and capture exit code
        if claude --dangerously-skip-permissions -p "$prompt"; then
            return 0
        fi

        local exit_code=$?

        # Check if it's a rate limit (exit codes vary, but we retry on any failure)
        if [ $attempt -lt $MAX_RETRIES ]; then
            echo "  ⚠️  Claude call failed (exit code: $exit_code)"
            echo "  ⏳ Waiting ${RETRY_DELAY}s before retry..."
            sleep $RETRY_DELAY
            # Exponential backoff
            RETRY_DELAY=$((RETRY_DELAY * 2))
        fi

        attempt=$((attempt + 1))
    done

    echo "  ❌ Failed after $MAX_RETRIES attempts"
    echo "  Pausing script. Resume with: ./scripts/run-all.sh"
    echo "  Or skip this task: echo '$CURRENT_TASK_KEY' >> .ralph-progress"
    exit 1
}

# Initialize git if needed
if [ ! -d ".git" ]; then
    git init -b main
    git add -A && git commit -m "Initial commit for $PROJECT"
fi

# Create starting tag
git add -A && git commit -m "Pre-ralph state" --allow-empty 2>/dev/null
git tag "ralph-start-$(date +%Y%m%d-%H%M%S)" 2>/dev/null

is_done() {
    grep -qxF "$1" "$PROGRESS_FILE" 2>/dev/null
}

mark_done() {
    echo "$1" >> "$PROGRESS_FILE"
}

# ============================================
# PHASE 0: VALIDATE EXISTING CONTENT
# ============================================
echo ""
echo "########################################################"
echo "### PHASE 0: VALIDATE EXISTING CONTENT (${#VALIDATE_TASKS[@]} files)"
echo "########################################################"

TASK_NUM=0
for task in "${VALIDATE_TASKS[@]}"; do
    TASK_NUM=$((TASK_NUM + 1))
    TASK_KEY="validate:$task"

    if is_done "$TASK_KEY"; then
        echo "✅ SKIP (done): $task"
        continue
    fi

    echo ""
    echo "============================================"
    echo "=== VALIDATE $TASK_NUM/${#VALIDATE_TASKS[@]}: $task ==="
    echo "============================================"

    TASK_NAME=$(basename "$task" .mdx)
    RESEARCH_FILE="research/${task%.mdx}.research.md"

    git tag "validate-$TASK_NUM-start" 2>/dev/null
    CURRENT_TASK_KEY="$TASK_KEY"

    echo ">>> Phase 1: RESEARCH"
    run_claude "You are working on the $PROJECT project in $PROJECT_DIR.

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
   - Note any corrections needed
   - Include source URLs for every fact
   - Keep it concise - bullet points preferred

Do NOT modify the content file yet. Research only.

After research:
git add -A && git commit -m '[$PROJECT] $TASK_NAME - Research complete'"

    echo ">>> Phase 2: IMPLEMENT"
    run_claude "You are working on the $PROJECT project in $PROJECT_DIR.

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
    run_claude "You are working on the $PROJECT project in $PROJECT_DIR.

Your task: Final review and update tracking.

Instructions:
1. Re-read the updated file: src/content/docs/$task
2. Verify:
   - last_validated: $TODAY is in frontmatter
   - Version numbers match research findings
   - Sources section exists with working URLs
   - Content is accurate and helpful
3. Fix any issues found
4. Update EXECUTION_PLAN.md: find the row for this file and change 🔴 to ✅

After review:
git add -A && git commit -m '[$PROJECT] $TASK_NAME - Reviewed and verified'"

    git tag "validate-$TASK_NUM-complete" 2>/dev/null
    mark_done "$TASK_KEY"
    echo "=== Validation of $task complete ==="
    sleep 2
done

# ============================================
# PHASE 1: CREATE NEW CONTENT
# ============================================
echo ""
echo "########################################################"
echo "### PHASE 1: CREATE NEW CONTENT (${#CREATE_TASKS[@]} files)"
echo "########################################################"

TASK_NUM=0
for task_spec in "${CREATE_TASKS[@]}"; do
    TASK_NUM=$((TASK_NUM + 1))

    # Parse task spec: "path|title|description"
    IFS='|' read -r task title description <<< "$task_spec"
    TASK_KEY="create:$task"

    if is_done "$TASK_KEY"; then
        echo "✅ SKIP (done): $task"
        continue
    fi

    echo ""
    echo "============================================"
    echo "=== CREATE $TASK_NUM/${#CREATE_TASKS[@]}: $title ==="
    echo "=== File: $task ==="
    echo "============================================"

    TASK_NAME=$(basename "$task" .mdx)
    RESEARCH_FILE="research/${task%.mdx}.research.md"

    # Ensure directory exists
    mkdir -p "$(dirname "$PROJECT_DIR/src/content/docs/$task")"
    mkdir -p "$(dirname "$PROJECT_DIR/$RESEARCH_FILE")"

    git tag "create-$TASK_NUM-start" 2>/dev/null

    echo ">>> Phase 1: RESEARCH"
    run_claude "You are working on the $PROJECT project in $PROJECT_DIR.

Your task: Research for NEW glossary entry: $title
File to create: src/content/docs/$task
Description: $description

Instructions:
1. Use WebSearch to research this topic thoroughly:
   - Official documentation (NVIDIA, ROS, academic sources)
   - Current best practices as of January 2026
   - How it relates to NVIDIA + ROS2 ecosystem
   - Prerequisites and related concepts
2. Create research notes at: $RESEARCH_FILE
   - Key concepts to cover
   - Current version numbers
   - Code examples to include
   - Diagrams needed
   - Source URLs for citations
3. Review existing entries in src/content/docs/ to match the style

Research only - do NOT create the content file yet.

After research:
git add -A && git commit -m '[$PROJECT] $TASK_NAME - Research complete'"

    echo ">>> Phase 2: IMPLEMENT"
    run_claude "You are working on the $PROJECT project in $PROJECT_DIR.

Your task: Create NEW glossary entry for: $title
File: src/content/docs/$task

Instructions:
1. Read your research notes: $RESEARCH_FILE
2. Read similar existing entries to match the style (e.g., src/content/docs/concepts/fundamentals/kinematics.mdx)
3. Create src/content/docs/$task with:

Frontmatter:
---
title: $title
description: $description
last_validated: $TODAY
sidebar:
  badge:
    text: Practical  # or Conceptual or Deep Dive
    variant: success # or note or tip
---

Content structure:
- Import Starlight components
- Level badge
- Clear one-paragraph definition
- Why it matters for robotics
- Key concepts with diagrams (ASCII art)
- Code examples (working, tested)
- Prerequisites (LinkCard)
- Related Terms (CardGrid)
- Sources section with citations

Keep it concise and practical. Quality over quantity.

After implementation:
git add -A && git commit -m '[$PROJECT] $TASK_NAME - New entry created'"

    echo ">>> Phase 3: REVIEW"
    run_claude "You are working on the $PROJECT project in $PROJECT_DIR.

Your task: Review new glossary entry: $title
File: src/content/docs/$task

Instructions:
1. Read the new file and verify:
   - Frontmatter is complete with last_validated
   - Content follows the established style
   - Code examples are correct
   - Prerequisites and Related Terms link to existing entries
   - Sources section has proper citations
2. Fix any issues
3. Add entry to EXECUTION_PLAN.md under 'New Entries Created' or update status

After review:
git add -A && git commit -m '[$PROJECT] $TASK_NAME - Reviewed and finalized'"

    git tag "create-$TASK_NUM-complete" 2>/dev/null
    mark_done "$TASK_KEY"
    echo "=== Creation of $title complete ==="
    sleep 2
done

# ============================================
# FINAL AUDIT
# ============================================
echo ""
echo "########################################################"
echo "### FINAL AUDIT"
echo "########################################################"

run_claude "You are working on the $PROJECT project in $PROJECT_DIR.

Perform a complete audit:

1. Count files in src/content/docs/ (excluding index, how-to-use, contributing)
2. Verify .ralph-progress has all tasks marked
3. Spot-check 3 random files for:
   - last_validated present
   - Sources section exists
4. Run: npm run build (to verify no errors)
5. Create VALIDATION_AUDIT.md with:
   - Total entries validated
   - New entries created
   - Any issues found
   - Build status
   - Recommendations for next iteration

Final commit: git add -A && git commit -m '[$PROJECT] Audit complete - $TODAY'"

git tag "ralph-complete-$(date +%Y%m%d-%H%M%S)" 2>/dev/null

echo ""
echo "########################################################"
echo "### $PROJECT Ralph Wiggum Loop Complete"
echo "########################################################"
echo ""
echo "Summary:"
echo "  Validated: ${#VALIDATE_TASKS[@]} existing files"
echo "  Created: ${#CREATE_TASKS[@]} new files"
echo ""
echo "Git history:"
git log --oneline -25
echo ""
echo "Rollback commands:"
echo "  git reset --hard validate-N-start  # Before validation N"
echo "  git reset --hard create-N-start    # Before creation N"
echo "  git reset --hard ralph-start-*     # Complete rollback"
echo "  rm .ralph-progress                 # Reset progress"
