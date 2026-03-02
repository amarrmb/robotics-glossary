# learn.devicenexus.ai — Design Document

## Goal

Build `learn.devicenexus.ai` as a unified physical AI learning hub that combines the community robotics glossary with DeviceNexus-authored thought leadership (Guides, Dispatch newsletter, Papers), deployed on Cloudflare Pages from a closed-source `devicenexus-docs` repo.

## Context

- `robotics-glossary` (OSS, `amarrmb/robotics-glossary`) was well-received publicly and drives discovery
- DeviceNexus has no public brand recognition yet — this hub builds it
- The content flywheel: Glossary (discover) → Guides/Papers (trust) → Dispatch (retain) → DeviceNexus.ai (convert)
- "The Physical AI Dispatch" = weekly newsletter reviewing the robotics world
- 0 Substack subscribers — clean slate, no migration needed

---

## Repository Split

| Repo | Visibility | Deploys to | Contains |
|------|-----------|------------|---------|
| `robotics-glossary` | OSS | `amarrmb.github.io/robotics-glossary` | Pure community reference only — Concepts, Hardware, Software |
| `devicenexus-docs` | Closed | `learn.devicenexus.ai` | Guides, Dispatch, Papers, About + consumes glossary via submodule |

The glossary is the source of truth for reference content. `devicenexus-docs` pulls it in at build time via git submodule.

---

## Information Architecture

### learn.devicenexus.ai URL structure

```
learn.devicenexus.ai/
├── /                     ← Landing: four equal-weight section CTAs
├── /glossary/            ← Community reference (synced from OSS repo)
│   ├── /concepts/...
│   ├── /hardware/...
│   └── /software/...
├── /guides/              ← DeviceNexus engineering deep dives
│   ├── /                 ← Guide index
│   ├── /act-training-lessons/
│   └── /latency-floor-model/
├── /dispatch/            ← The Physical AI Dispatch
│   ├── /                 ← Subscribe + issue archive
│   ├── /2026-W11/        ← Individual issues
│   └── feed.xml          ← RSS feed
├── /papers/              ← Research-quality work
│   ├── /                 ← Papers index
│   └── /latency-floor-model/  ← First paper
└── /about/               ← Amar Balutkar + DeviceNexus
```

### Navigation

Four equal tabs in the site header:

```
[Glossary]  [Guides]  [Dispatch]  [Papers]
```

- **Glossary** — Starlight sidebar, reference mode, neutral community feel
- **Guides** — full-width article layout, no sidebar, DeviceNexus byline
- **Dispatch** — article layout, subscribe form, RSS link, issue archive
- **Papers** — full-width article layout, citeable, version-stamped

When on Guides / Dispatch / Papers: reference sidebar hidden, full-width reading column.
When on Glossary: sidebar visible, header shows "← Learn hub" link instead of tabs.

---

## devicenexus-docs Directory Structure

```
devicenexus-docs/
├── submodules/
│   └── robotics-glossary/         ← git submodule (OSS repo)
├── scripts/
│   └── sync-glossary.mjs          ← pre-build: copies submodule → src/content/docs/glossary/
├── src/
│   ├── content/
│   │   └── docs/
│   │       ├── glossary/          ← synced from submodule at build time (gitignored)
│   │       ├── guides/            ← moved from robotics-glossary
│   │       │   ├── index.mdx
│   │       │   ├── act-training-lessons.mdx
│   │       │   └── latency-floor-model.mdx
│   │       ├── dispatch/          ← new
│   │       │   └── index.mdx      ← subscribe + archive
│   │       ├── papers/            ← new
│   │       │   └── index.mdx
│   │       └── about.mdx          ← new
│   ├── pages/
│   │   └── dispatch/
│   │       └── feed.xml.js        ← RSS feed endpoint
│   └── styles/
│       └── custom.css             ← DeviceNexus branding
└── astro.config.mjs               ← unified config
```

### sync-glossary.mjs

Runs as `prebuild` in `package.json`. Copies `submodules/robotics-glossary/src/content/docs/**` → `src/content/docs/glossary/**`. ~20 lines of Node.js `fs` calls. No external dependencies.

When glossary updates: `git submodule update --remote && git commit -m "chore: update glossary"` in devicenexus-docs triggers a Cloudflare Pages rebuild.

---

## robotics-glossary Cleanup

### Remove from robotics-glossary

- `src/content/docs/guides/` directory (all guide MDX files move to devicenexus-docs)
- `Guides` sidebar section in `astro.config.mjs`
- `remarkMath` + `rehypeKatex` plugins (only needed for latency guide)
- `remark-math` and `rehype-katex` npm packages

### Keep in robotics-glossary

- All Concepts, Hardware, Software, Contributing content
- `learning-path.mdx` — purely educational, no DN branding
- "In Practice" callouts — update links from `/robotics-glossary/guides/...` to `https://learn.devicenexus.ai/guides/...`
- `how-to-use.mdx`, `index.mdx`

After cleanup, robotics-glossary is a clean, neutral reference with zero DeviceNexus branding in the sidebar.

---

## Content Ownership & Edit Permissions

| Section | Edit link | Rationale |
|---------|-----------|-----------|
| Glossary | ✅ Points to OSS repo on GitHub | Community contributions welcome |
| Guides | ❌ `editUrl: false` | DeviceNexus authored |
| Dispatch | ❌ `editUrl: false` | DeviceNexus authored |
| Papers | ❌ `editUrl: false` | Citeable, canonical, must not drift |

Papers additionally get:
- "Published by DeviceNexus" notice with version + date stamp
- No "suggest edits" affordance anywhere on page
- Future: arXiv preprint link for external citability

`editUrl: false` is set via frontmatter on each Guides/Dispatch/Papers file, or via the content collection schema default so it cannot be forgotten.

---

## The Physical AI Dispatch

- Each issue: `dispatch/2026-W11.mdx` → renders at `learn.devicenexus.ai/dispatch/2026-W11/`
- `/dispatch/feed.xml` — Astro `@astrojs/rss` endpoint, auto-generated from dispatch content collection
- **Email delivery: Buttondown** (free ≤100 subscribers, watches RSS, auto-sends on new issue)
- Subscribe form = Buttondown embed `<form>` on `/dispatch/` index page
- Workflow: write MDX → push → Cloudflare Pages builds → Buttondown detects RSS update → delivers email

---

## Author Identity

Every Guide, Dispatch issue, and Paper gets:
- Byline: "Amar Balutkar · DeviceNexus"
- Publication date + reading time
- Author card at bottom linking to `/about/`

The `/about/` page ties personal brand to company: who Amar is, what DeviceNexus builds, links to all published work.

---

## Deployment: Cloudflare Pages

- DNS already at Cloudflare → `learn.devicenexus.ai` custom domain is two clicks
- Build command: `node scripts/sync-glossary.mjs && astro build`
- Cloudflare Pages clones with `--recurse-submodules` automatically
- Environment variables: `PUBLIC_POSTHOG_KEY`, `PUBLIC_POSTHOG_HOST`
- `amarrmb.github.io/robotics-glossary` continues to work (OSS site stays live)

---

## Visual Identity

| Section | Accent | Layout | Sidebar |
|---------|--------|--------|---------|
| Glossary | Indigo (current) | Starlight standard | Full reference sidebar |
| Guides | Amber/orange | Full-width article | None |
| Dispatch | Amber/orange | Full-width article | None |
| Papers | Amber/orange | Full-width article | None |

Guides/Dispatch/Papers share DeviceNexus amber. Glossary stays indigo (community neutral).

---

## Cross-linking Model

| From | To | Mechanism |
|------|-----|-----------|
| Glossary term | Related guide | `<Aside title="In Practice">` (already in place, update URLs) |
| Guide | Glossary terms used | "Key Concepts" `<CardGrid>` at bottom of each guide |
| Papers | Glossary terms | "Key Concepts" section |
| Dispatch issue | Glossary / Guides | Inline links throughout issue text |
| Homepage | All four sections | Equal-weight card grid |

---

## Open Questions (Post-MVP)

- arXiv preprint for latency floor model paper
- `learn.devicenexus.ai` link from `devicenexus.ai` product site
- GTC March 16 as public launch moment
- Series/collections grouping (e.g. "Building with ACT" groups guide + related glossary terms)
