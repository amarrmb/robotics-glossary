---
title: Style Guide
description: Conventions for writing glossary entries
---

Follow these guidelines to maintain consistency across the glossary.

## Entry Structure

Every term should follow this structure:

```markdown
---
title: Term Name
description: One-line summary (appears in search)
sidebar:
  badge:
    text: Difficulty/Version
    variant: success|caution|note
---

import { Aside, Card, CardGrid, LinkCard } from '@astrojs/starlight/components';

**Term Name** is [one-paragraph definition]. [Why it matters for robotics].

<Aside type="tip|note|caution">
Key insight or common misconception
</Aside>

## Prerequisites (if applicable)
<CardGrid>
  <LinkCard title="..." href="..." />
</CardGrid>

## Main Content
[Detailed explanation with subsections]

## Code Example (if applicable)
[Working code snippet]

## Related Terms
<CardGrid>
  <LinkCard title="..." href="..." />
</CardGrid>

## Learn More
- [External Link](url) — Brief description
```

## Writing Style

### Tone
- **Direct and practical** — Get to the point
- **Accessible** — Explain jargon when first used
- **Technically accurate** — No hand-waving
- **Consistent** — Use the same terms throughout

### Formatting
- **Bold** the term name in the first sentence
- Use **headings** (##) to break up content
- Include **code examples** where relevant
- Add **diagrams** for complex concepts

### Length
- **Summary**: 1-2 sentences
- **Main content**: 200-500 words typical
- **Code examples**: Complete and runnable

## Badges

Use sidebar badges to indicate:

| Badge | Variant | Use For |
|-------|---------|---------|
| `Beginner` | `success` (green) | No prior knowledge needed |
| `Intermediate` | `caution` (yellow) | Assumes basics |
| `Advanced` | `danger` (red) | Expert-level |
| `NVIDIA` | `note` (blue) | NVIDIA-specific terms |
| `v3.2` | `note` (blue) | Version-specific |

## Code Examples

### Do
```python
# Clear, complete example that actually runs
import numpy as np

def forward_kinematics(theta1, theta2, L1=1.0, L2=1.0):
    """Compute 2-link arm end-effector position."""
    x = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    y = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)
    return x, y
```

### Don't
```python
# Incomplete snippet that won't run
x = compute_fk(...)  # What is compute_fk?
```

## Diagrams

Use ASCII art for simple diagrams:

```
Input → [Process] → Output
```

For complex diagrams:
- Create SVG files in `src/assets/diagrams/`
- Reference with `![Alt text](../../assets/diagrams/file.svg)`

## Links

### Internal Links
```markdown
[Term Name](/robotics-glossary/category/term/)
```

### External Links
```markdown
[NVIDIA Docs](https://developer.nvidia.com/...) — Official reference
```

Always include a brief description of external links.

## Asides

Use Starlight's `<Aside>` component for callouts:

```jsx
<Aside type="tip">
Helpful suggestion or best practice
</Aside>

<Aside type="note">
Additional context or clarification
</Aside>

<Aside type="caution">
Common pitfall or limitation
</Aside>

<Aside type="danger">
Critical warning — something could break
</Aside>
```

## Related Terms

Always include a "Related Terms" section with 2-4 links using `<CardGrid>` and `<LinkCard>`:

```jsx
<CardGrid>
  <LinkCard
    title="Related Term"
    description="Brief description"
    href="/robotics-glossary/category/term/"
  />
</CardGrid>
```

## Version Tracking

For NVIDIA-specific terms, include version information:

```jsx
<Aside type="note" title="Current Version">
Isaac ROS 3.2 — Aligned with JetPack 6.1
</Aside>
```

Update these when new versions release.
