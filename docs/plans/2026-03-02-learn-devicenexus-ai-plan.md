# learn.devicenexus.ai Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Restructure `devicenexus-docs` into a unified `learn.devicenexus.ai` hub (Glossary + Guides + Dispatch + Papers) with custom 4-tab navigation, article layout for thought leadership content, glossary submodule integration, RSS feed, and Cloudflare Pages deployment — while cleaning up `robotics-glossary` to be pure community reference.

**Architecture:** Two repos: `robotics-glossary` (OSS, stays at GitHub Pages) is consumed as a git submodule by `devicenexus-docs` (closed, deploys to `learn.devicenexus.ai`). A pre-build sync script copies the submodule content into the Starlight docs directory. A custom Starlight Header component provides 4-tab navigation. Guides/Dispatch/Papers use `template: splash` for full-width article layout; Glossary uses the standard Starlight sidebar.

**Tech Stack:** Astro 5, `@astrojs/starlight` 0.37, `@astrojs/rss`, Buttondown (email), Cloudflare Pages, git submodules, Node.js (sync script)

---

## Overview of Changes

### Repo 1: `robotics-glossary` (OSS — `/home/amar/baskd/robotics-glossary/`)
- Remove `src/content/docs/guides/` (guides move to devicenexus-docs)
- Remove Guides sidebar section from `astro.config.mjs`
- Remove `remark-math`, `rehype-katex` (only needed for latency guide)
- Update "In Practice" callout links from `/robotics-glossary/guides/...` → `https://learn.devicenexus.ai/guides/...`

### Repo 2: `devicenexus-docs` (Closed — `/home/amar/baskd/devicenexus-docs/`)
- Rename `blog/` → `guides/`, rename `research/` → `papers/`
- Add `robotics-glossary` as git submodule
- Add `scripts/sync-glossary.mjs` pre-build script
- Add `dispatch/` content section (newsletter)
- Custom 4-tab `Header.astro` component
- Article layout + byline on all Guides/Dispatch/Papers pages
- `editUrl: false` on all DN-authored content
- RSS feed at `/dispatch/feed.xml`
- Buttondown subscribe form
- Cloudflare Pages deployment config

---

## Phase A: robotics-glossary Cleanup

### Task 1: Remove guides from robotics-glossary

**Repo:** `robotics-glossary`

**Files:**
- Delete: `src/content/docs/guides/` (entire directory)
- Delete: `src/content/docs/learning-path.mdx` (DN content — moves to devicenexus-docs)
- Modify: `astro.config.mjs`
- Modify: All `*.mdx` files containing "In Practice" links to `/robotics-glossary/guides/`

**Step 1: Remove the guides directory and learning-path**

```bash
rm -rf src/content/docs/guides/
rm src/content/docs/learning-path.mdx
```

**Step 2: Remove the Guides sidebar section and math plugins from astro.config.mjs**

In `astro.config.mjs`, remove:
- The entire `{ label: 'Guides', ... }` sidebar group
- The `{ label: 'Learning Path', slug: 'learning-path' }` item from the Start Here group
- The `import remarkMath from 'remark-math'` line
- The `import rehypeKatex from 'rehype-katex'` line
- The entire `markdown: { remarkPlugins: [remarkMath], rehypePlugins: [rehypeKatex] }` block at the bottom

The Start Here section should become:
```js
{
  label: 'Start Here',
  items: [
    { label: 'Introduction', slug: 'index' },
    { label: 'How to Use', slug: 'how-to-use' },
  ],
},
```

**Step 3: Update "In Practice" links across all concept MDX files**

These files contain `href="/robotics-glossary/guides/...` that need updating:
```bash
grep -rl "/robotics-glossary/guides/" src/content/docs/
```

For each match, change:
- `/robotics-glossary/guides/act-training-lessons/` → `https://learn.devicenexus.ai/guides/act-training-lessons/`
- `/robotics-glossary/guides/latency-floor-model/` → `https://learn.devicenexus.ai/guides/latency-floor-model/`

Also update `src/content/docs/index.mdx` — remove any cards linking to guides, and remove the "Learning Path" card (it linked to `learning-path` which is deleted).

**Step 4: Remove math packages**

```bash
npm uninstall remark-math rehype-katex
```

**Step 5: Verify build passes**

```bash
npm run build
```

Expected: Clean build with 0 errors. Page count will drop (guides removed). No broken internal links.

**Step 6: Commit**

```bash
git add -A
git commit -m "chore: remove guides from OSS repo — moved to devicenexus-docs"
```

---

## Phase B: devicenexus-docs — Content Structure

**All remaining tasks are in `/home/amar/baskd/devicenexus-docs/`**

### Task 2: Rename blog/ → guides/, research/ → papers/

**Files:**
- Rename: `src/content/docs/blog/` → `src/content/docs/guides/`
- Rename: `src/content/docs/research/` → `src/content/docs/papers/`
- Modify: `astro.config.mjs` (update sidebar slugs)

**Step 1: Rename directories**

```bash
cd /home/amar/baskd/devicenexus-docs
mv src/content/docs/blog src/content/docs/guides
mv src/content/docs/research src/content/docs/papers
```

**Step 2: Update astro.config.mjs sidebar to match new paths**

Change the sidebar from:
```js
{ label: 'Blog', items: [...] }
{ label: 'Research', items: [...] }
```

To:
```js
{
  label: 'Guides',
  items: [
    { label: 'ACT Training Lessons', slug: 'guides/act-training-lessons' },
    { label: 'Latency Floor Model', slug: 'guides/latency-floor-model' },
    { label: 'Voice + Vision on Jetson', slug: 'guides/jetson-assistant' },
  ],
},
{
  label: 'Papers',
  items: [
    { label: 'Latency Floor Model', slug: 'papers/latency-floor-model' },
  ],
},
```

**Step 3: Verify build**

```bash
npm run build
```

Expected: clean build, pages now at `/guides/...` and `/papers/...`

**Step 4: Commit**

```bash
git add -A
git commit -m "refactor: rename blog→guides, research→papers"
```

---

### Task 3: Add robotics-glossary as git submodule + sync script

**Files:**
- Create: `.gitmodules` (auto-generated by git)
- Create: `scripts/sync-glossary.mjs`
- Modify: `package.json` (add prebuild script)
- Modify: `.gitignore` (ignore synced glossary content)

**Step 1: Add the submodule**

```bash
cd /home/amar/baskd/devicenexus-docs
git submodule add https://github.com/amarrmb/robotics-glossary.git submodules/robotics-glossary
```

This creates `submodules/robotics-glossary/` and `.gitmodules`.

**Step 2: Create `scripts/sync-glossary.mjs`**

```js
// scripts/sync-glossary.mjs
// Copies robotics-glossary content into src/content/docs/glossary/ before build.
// Run automatically via "prebuild" in package.json.

import { cpSync, rmSync, existsSync, mkdirSync } from 'fs';
import { join } from 'path';
import { fileURLToPath } from 'url';
import { dirname } from 'path';

const __dirname = dirname(fileURLToPath(import.meta.url));
const root = join(__dirname, '..');

const src = join(root, 'submodules/robotics-glossary/src/content/docs');
const dest = join(root, 'src/content/docs/glossary');

if (!existsSync(src)) {
  console.error('ERROR: submodules/robotics-glossary not found. Run: git submodule update --init');
  process.exit(1);
}

// Clean and re-copy
if (existsSync(dest)) rmSync(dest, { recursive: true });
mkdirSync(dest, { recursive: true });
cpSync(src, dest, { recursive: true });

console.log(`Synced glossary content → ${dest}`);
```

**Step 3: Add prebuild hook to `package.json`**

Add `"prebuild"` and `"predev"` scripts so sync runs automatically:

```json
{
  "scripts": {
    "predev": "node scripts/sync-glossary.mjs",
    "dev": "astro dev",
    "prebuild": "node scripts/sync-glossary.mjs",
    "build": "astro build",
    "preview": "astro preview"
  }
}
```

**Step 4: Add synced glossary to `.gitignore`**

Create or append to `.gitignore`:
```
src/content/docs/glossary/
```

**Step 5: Test sync runs**

```bash
node scripts/sync-glossary.mjs
```

Expected output: `Synced glossary content → .../src/content/docs/glossary`

Verify: `ls src/content/docs/glossary/` should show `concepts/`, `hardware/`, `software/`, etc.

**Step 6: Add glossary to astro.config.mjs sidebar**

Add the glossary sidebar group to `astro.config.mjs`. This goes ABOVE the Guides/Papers groups:

```js
{
  label: 'Glossary',
  collapsed: false,
  items: [
    {
      label: 'Concepts',
      collapsed: false,
      items: [
        { label: 'Fundamentals', autogenerate: { directory: 'glossary/concepts/fundamentals' } },
        { label: 'Perception', autogenerate: { directory: 'glossary/concepts/perception' } },
        { label: 'Control & Planning', autogenerate: { directory: 'glossary/concepts/control' } },
        { label: 'Learning from Demonstration', autogenerate: { directory: 'glossary/concepts/learning' } },
        { label: 'AI & Learning', autogenerate: { directory: 'glossary/concepts/ai' } },
      ],
    },
    {
      label: 'Hardware',
      autogenerate: { directory: 'glossary/hardware' },
    },
    {
      label: 'Software',
      autogenerate: { directory: 'glossary/software' },
    },
  ],
},
```

**Step 7: Verify build with glossary**

```bash
npm run build
```

Expected: glossary pages appear at `/glossary/concepts/...`, `/glossary/hardware/...`, etc. No errors.

**Step 8: Commit**

```bash
git add -A
git commit -m "feat: add robotics-glossary submodule + sync script"
```

---

### Task 4: Add `dispatch/` section

**Files:**
- Create: `src/content/docs/dispatch/index.mdx`
- Create: `src/content/docs/dispatch/2026-W10.mdx` (first placeholder issue)
- Modify: `astro.config.mjs` (add Dispatch sidebar group)

**Step 1: Create `src/content/docs/dispatch/index.mdx`**

```mdx
---
title: The Physical AI Dispatch
description: A weekly review of what's happening in physical AI and robotics — by Amar Balutkar, DeviceNexus
editUrl: false
template: splash
---

import { LinkCard, CardGrid } from '@astrojs/starlight/components';

**A weekly review of what's happening in the world of physical AI.**

Every week: notable papers, hardware releases, research from the community, and what it means for practitioners building with robots.

— Amar Balutkar, [DeviceNexus](https://devicenexus.ai)

---

## Subscribe

Get each issue delivered to your inbox. No spam, no tracking, unsubscribe any time.

<div class="dispatch-subscribe">
  <!-- Buttondown embed goes here in Task 8 -->
  <p><em>Subscribe form coming soon — <a href="/dispatch/feed.xml">RSS feed available now</a></em></p>
</div>

---

## Issues

<CardGrid>
  <LinkCard
    title="Issue #1 — Week of March 10, 2026"
    description="First issue placeholder"
    href="/dispatch/2026-W10/"
  />
</CardGrid>
```

**Step 2: Create `src/content/docs/dispatch/2026-W10.mdx`**

```mdx
---
title: "The Physical AI Dispatch — Issue #1"
description: Week of March 10, 2026 — notable papers and releases in physical AI
editUrl: false
template: splash
---

*Week of March 10, 2026 · by Amar Balutkar, DeviceNexus*

---

First issue. More to come.
```

**Step 3: Add Dispatch to sidebar in `astro.config.mjs`**

```js
{
  label: 'Dispatch',
  items: [
    { label: 'Subscribe + Archive', slug: 'dispatch' },
    { label: 'Issue #1 — Mar 10', slug: 'dispatch/2026-W10' },
  ],
},
```

**Step 4: Verify build**

```bash
npm run build
```

Expected: `/dispatch/` and `/dispatch/2026-W10/` pages build cleanly.

**Step 5: Commit**

```bash
git add -A
git commit -m "feat: add Dispatch section with placeholder issue"
```

---

## Phase C: Custom 4-Tab Header

### Task 5: Custom Header component

**Files:**
- Create: `src/components/Header.astro`
- Modify: `astro.config.mjs` (register component override)

**Step 1: Create `src/components/Header.astro`**

The custom header renders the standard Starlight header PLUS a 4-tab nav bar below it. Active tab is determined by the current URL path.

```astro
---
// src/components/Header.astro
// Adds 4-tab nav below the standard Starlight header.
import Default from '@astrojs/starlight/components/Header.astro';

const pathname = Astro.url.pathname;

// Determine active section from URL
function getActiveTab(path: string): string {
  if (path.includes('/glossary/') || path.includes('/glossary')) return 'glossary';
  if (path.includes('/guides/') || path.includes('/guides')) return 'guides';
  if (path.includes('/dispatch/') || path.includes('/dispatch')) return 'dispatch';
  if (path.includes('/papers/') || path.includes('/papers')) return 'papers';
  return '';
}

const activeTab = getActiveTab(pathname);

const tabs = [
  { id: 'glossary', label: 'Glossary', href: '/glossary/concepts/fundamentals/kinematics/' },
  { id: 'guides', label: 'Guides', href: '/guides/' },
  { id: 'dispatch', label: 'Dispatch', href: '/dispatch/' },
  { id: 'papers', label: 'Papers', href: '/papers/' },
];
---

<Default {...Astro.props}><slot slot="before-title" name="before-title" /></Default>

<nav class="dn-section-tabs" aria-label="Site sections">
  <div class="dn-section-tabs__inner">
    {tabs.map(tab => (
      <a
        href={tab.href}
        class:list={['dn-section-tabs__tab', { 'dn-section-tabs__tab--active': activeTab === tab.id }]}
        aria-current={activeTab === tab.id ? 'page' : undefined}
      >
        {tab.label}
      </a>
    ))}
  </div>
</nav>

<style>
  .dn-section-tabs {
    background: var(--sl-color-bg-nav);
    border-bottom: 1px solid var(--sl-color-hairline);
    padding: 0 var(--sl-nav-pad-x);
  }

  .dn-section-tabs__inner {
    display: flex;
    gap: 0;
    max-width: var(--sl-content-width);
    margin: 0 auto;
  }

  .dn-section-tabs__tab {
    display: inline-block;
    padding: 0.6rem 1.2rem;
    font-size: var(--sl-text-sm);
    font-weight: 500;
    color: var(--sl-color-text-accent);
    text-decoration: none;
    border-bottom: 2px solid transparent;
    transition: border-color 0.15s, color 0.15s;
  }

  .dn-section-tabs__tab:hover {
    color: var(--sl-color-white);
    border-bottom-color: var(--sl-color-text-accent);
  }

  .dn-section-tabs__tab--active {
    color: var(--sl-color-white);
    border-bottom-color: var(--sl-color-accent);
    font-weight: 600;
  }
</style>
```

**Step 2: Register the override in `astro.config.mjs`**

In the `starlight({...})` config, add:

```js
components: {
  Header: './src/components/Header.astro',
},
```

**Step 3: Verify dev server shows tabs**

```bash
npm run dev
```

Open `http://localhost:4321` in a browser. Expected: 4 tabs appear below the standard Starlight header. Navigating to `/guides/` highlights the Guides tab. Navigating to `/glossary/...` highlights the Glossary tab.

**Step 4: Commit**

```bash
git add -A
git commit -m "feat: add 4-tab section navigation header"
```

---

## Phase D: Article Layout + Author Bylines

### Task 6: Article layout and editUrl: false on all DN-authored pages

Guides, Dispatch, and Papers pages need: full-width article layout (`template: splash` in frontmatter), `editUrl: false`, and a visible author byline. Glossary pages retain the standard sidebar layout and DO get edit links (pointing to the OSS repo).

**Files:**
- Modify: All `src/content/docs/guides/*.mdx` frontmatter
- Modify: All `src/content/docs/dispatch/*.mdx` frontmatter
- Modify: All `src/content/docs/papers/*.mdx` frontmatter
- Create: `src/components/AuthorByline.astro`
- Modify: `src/styles/custom.css` (article layout styles)

**Step 1: Add required frontmatter to all guides/**

For each file in `src/content/docs/guides/`:

```yaml
---
title: "..."
description: "..."
editUrl: false
template: splash
---
```

Files to update:
- `guides/act-training-lessons.mdx`
- `guides/latency-floor-model.mdx`
- `guides/jetson-assistant.mdx`
- `guides/index.mdx`

**Step 2: Add required frontmatter to all dispatch/ and papers/**

Same two frontmatter fields to all files in `dispatch/` and `papers/`.

**Step 3: Add author byline below title on each guide/paper**

At the top of the content body (after frontmatter) on each Guides, Papers, and Dispatch issue file, add:

```mdx
<div class="dn-byline">Amar Balutkar · <a href="https://devicenexus.ai">DeviceNexus</a> · <time>March 2026</time></div>
```

Update the date per article. Dispatch issues also show "Issue #N".

**Step 4: Add article layout styles to `src/styles/custom.css`**

Append to `custom.css`:

```css
/* Article layout for Guides, Dispatch, Papers (template: splash pages) */
.dn-byline {
  font-size: var(--sl-text-sm);
  color: var(--sl-color-gray-3);
  margin-bottom: 2rem;
  padding-bottom: 1rem;
  border-bottom: 1px solid var(--sl-color-hairline);
}

.dn-byline a {
  color: var(--sl-color-text-accent);
  text-decoration: none;
}

/* Constrain article width for readability on splash pages */
.sl-markdown-content {
  max-width: 75ch;
}
```

**Step 5: Verify build and visual check**

```bash
npm run build && npm run preview
```

Open a guide page. Expected:
- No sidebar
- Full-width reading column
- Byline visible below title
- No "Edit this page" link in footer
- Reading width is comfortable (not full viewport width)

**Step 6: Commit**

```bash
git add -A
git commit -m "feat: article layout + editUrl:false + bylines on Guides/Dispatch/Papers"
```

---

## Phase E: RSS Feed

### Task 7: Add `/dispatch/feed.xml` RSS endpoint

**Files:**
- Create: `src/pages/dispatch/feed.xml.ts`
- Install: `@astrojs/rss` (may already be installed — check `package.json`)

**Step 1: Check if @astrojs/rss is installed**

```bash
cat package.json | grep rss
```

If not present:
```bash
npm install @astrojs/rss
```

**Step 2: Create `src/pages/dispatch/feed.xml.ts`**

```ts
// src/pages/dispatch/feed.xml.ts
import rss from '@astrojs/rss';
import { getCollection } from 'astro:content';
import type { APIContext } from 'astro';

export async function GET(context: APIContext) {
  const issues = await getCollection('docs', (entry) =>
    entry.id.startsWith('dispatch/') && entry.id !== 'dispatch/index'
  );

  // Sort by slug descending (newest first)
  issues.sort((a, b) => b.id.localeCompare(a.id));

  return rss({
    title: 'The Physical AI Dispatch',
    description: 'A weekly review of what\'s happening in physical AI and robotics — by Amar Balutkar, DeviceNexus',
    site: context.site ?? 'https://learn.devicenexus.ai',
    items: issues.map((issue) => ({
      title: issue.data.title,
      description: issue.data.description,
      pubDate: new Date(issue.data.last_validated ?? '2026-01-01'),
      link: `/dispatch/${issue.slug.replace('dispatch/', '')}/`,
    })),
    customData: `<language>en-us</language>`,
  });
}
```

**Step 3: Add `site` to `astro.config.mjs`**

At the top level of `defineConfig`:

```js
export default defineConfig({
  site: 'https://learn.devicenexus.ai',
  // ...
});
```

**Step 4: Update dispatch/index.mdx to link to real RSS feed**

Replace the placeholder subscribe section with:

```mdx
<div class="dispatch-subscribe">
  <p>📡 <a href="/dispatch/feed.xml">RSS Feed</a> — subscribe in any RSS reader</p>
</div>
```

(Buttondown embed will be added in Task 8)

**Step 5: Verify RSS feed generates**

```bash
npm run build
cat dist/dispatch/feed.xml | head -30
```

Expected: valid XML with `<channel>` and `<item>` elements for each dispatch issue.

**Step 6: Commit**

```bash
git add -A
git commit -m "feat: add RSS feed for The Physical AI Dispatch"
```

---

## Phase F: Email Subscribe (Buttondown)

### Task 8: Buttondown email subscription form

**Context:** Buttondown (buttondown.com) is the email provider. You need to create a free account at buttondown.com and get your username. The embed is a simple HTML form — no API key needed for the subscribe form itself.

**Files:**
- Modify: `src/content/docs/dispatch/index.mdx`
- Modify: `src/styles/custom.css`

**Step 1: Create Buttondown account**

1. Go to `https://buttondown.com`
2. Sign up with the DeviceNexus email
3. Note your Buttondown username (e.g. `physicalaI` or `devicenexus`)
4. In Buttondown settings → RSS: paste `https://learn.devicenexus.ai/dispatch/feed.xml` so Buttondown auto-sends when new issues publish

**Step 2: Replace the subscribe placeholder in `dispatch/index.mdx`**

Replace the subscribe section with the Buttondown embed form. Use your actual Buttondown username in the form action URL:

```mdx
<div class="dispatch-subscribe">
  <form
    action="https://buttondown.com/api/emails/embed-subscribe/YOUR_USERNAME"
    method="post"
    target="popupwindow"
    onsubmit="window.open('https://buttondown.com/YOUR_USERNAME', 'popupwindow')"
    class="dispatch-subscribe__form"
  >
    <input type="email" name="email" placeholder="you@example.com" required class="dispatch-subscribe__input" />
    <button type="submit" class="dispatch-subscribe__btn">Subscribe</button>
  </form>
  <p class="dispatch-subscribe__note">Weekly. No spam. <a href="/dispatch/feed.xml">RSS also available.</a></p>
</div>
```

Replace `YOUR_USERNAME` with your actual Buttondown username in both places.

**Step 3: Add subscribe form styles to `src/styles/custom.css`**

```css
/* Dispatch subscribe form */
.dispatch-subscribe {
  background: var(--sl-color-bg-sidebar);
  border: 1px solid var(--sl-color-hairline);
  border-radius: 8px;
  padding: 1.5rem;
  margin: 2rem 0;
  max-width: 480px;
}

.dispatch-subscribe__form {
  display: flex;
  gap: 0.5rem;
  flex-wrap: wrap;
}

.dispatch-subscribe__input {
  flex: 1;
  min-width: 220px;
  padding: 0.6rem 0.9rem;
  border: 1px solid var(--sl-color-hairline);
  border-radius: 6px;
  background: var(--sl-color-bg);
  color: var(--sl-color-text);
  font-size: var(--sl-text-sm);
}

.dispatch-subscribe__btn {
  padding: 0.6rem 1.2rem;
  background: var(--sl-color-accent);
  color: white;
  border: none;
  border-radius: 6px;
  font-size: var(--sl-text-sm);
  font-weight: 600;
  cursor: pointer;
}

.dispatch-subscribe__btn:hover {
  opacity: 0.9;
}

.dispatch-subscribe__note {
  margin-top: 0.75rem;
  font-size: var(--sl-text-xs);
  color: var(--sl-color-gray-3);
}
```

**Step 4: Verify form renders**

```bash
npm run dev
```

Open `http://localhost:4321/dispatch/`. Expected: subscribe form with email input + button, styled cleanly, RSS link below it.

**Step 5: Commit**

```bash
git add -A
git commit -m "feat: add Buttondown email subscribe form to Dispatch"
```

---

## Phase G: Site Config + Homepage

### Task 9: Site config, base URL, homepage

**Files:**
- Modify: `astro.config.mjs` (add `site`, clean up title/description)
- Modify: `src/content/docs/index.mdx` (four-panel landing)

**Step 1: Update `astro.config.mjs` top-level config**

```js
export default defineConfig({
  site: 'https://learn.devicenexus.ai',
  // no `base` — deploying at root
  integrations: [
    starlight({
      title: 'DeviceNexus Learn',
      description: 'Physical AI learning hub — Glossary, Guides, Newsletter, and Research by DeviceNexus',
      // ... rest of config
    }),
  ],
});
```

**Step 2: Rewrite `src/content/docs/index.mdx`**

```mdx
---
title: Physical AI Learning Hub
description: The canonical glossary, engineering guides, weekly newsletter, and research papers for physical AI practitioners
template: splash
editUrl: false
hero:
  title: Physical AI Learning Hub
  tagline: Robotics concepts, engineering deep dives, weekly dispatch, and research — by DeviceNexus
---

import { LinkCard, CardGrid } from '@astrojs/starlight/components';

<CardGrid>
  <LinkCard
    title="Glossary"
    description="Community reference for robotics concepts, hardware, and software. 50+ entries from fundamentals to VLA models."
    href="/glossary/concepts/fundamentals/kinematics/"
  />
  <LinkCard
    title="Guides"
    description="First-person engineering deep dives. Real failures, real data, real lessons from building with physical AI."
    href="/guides/"
  />
  <LinkCard
    title="The Physical AI Dispatch"
    description="Weekly review of papers, releases, and developments in the robotics world. Subscribe via email or RSS."
    href="/dispatch/"
  />
  <LinkCard
    title="Papers"
    description="Research-quality technical writing from DeviceNexus. Citeable, stable, opinionated."
    href="/papers/"
  />
</CardGrid>
```

**Step 3: Verify build**

```bash
npm run build
```

Expected: Homepage renders with 4-panel card grid. All 4 tabs work. Build is clean.

**Step 4: Commit**

```bash
git add -A
git commit -m "feat: four-panel homepage + site config for learn.devicenexus.ai"
```

---

### Task 10: About page

**Files:**
- Create: `src/content/docs/about.mdx`
- Modify: `astro.config.mjs` (add About to sidebar footer or standalone)

**Step 1: Create `src/content/docs/about.mdx`**

```mdx
---
title: About
description: Amar Balutkar and DeviceNexus — building infrastructure for physical AI
editUrl: false
template: splash
---

## Amar Balutkar

I'm building DeviceNexus — infrastructure for robotics companies to go from "robot can't do anything" to "robot succeeds in production and keeps getting better."

Before that: [fill in your background here].

---

## DeviceNexus

DeviceNexus provides the operational layer that connects training, deployment, and continuous improvement for physical AI systems.

We're not a model company, a simulation company, or a hardware company. We're the plumbing that connects them into a working pipeline.

[Learn more at devicenexus.ai →](https://devicenexus.ai)

---

## This Site

**[Glossary](/glossary/concepts/fundamentals/kinematics/)** — A community-maintained reference for robotics concepts. Open source on [GitHub](https://github.com/amarrmb/robotics-glossary).

**[Guides](/guides/)** — Engineering stories from building DeviceNexus. First-person accounts of what worked, what failed, and what we learned.

**[The Physical AI Dispatch](/dispatch/)** — Weekly review of the physical AI landscape. Papers, releases, commentary.

**[Papers](/papers/)** — Research-quality technical writing with citeable canonical URLs.
```

**Step 2: Add About to sidebar in `astro.config.mjs`**

Add as a standalone item at the bottom of the sidebar array:

```js
{ label: 'About', slug: 'about' },
```

**Step 3: Verify**

```bash
npm run build
```

Expected: `/about/` page builds cleanly.

**Step 4: Commit**

```bash
git add -A
git commit -m "feat: add About page"
```

---

## Phase H: Cloudflare Pages Deployment

### Task 11: Configure Cloudflare Pages

**Context:** devicenexus-docs is a private GitHub repo. Cloudflare Pages connects to it for CI/CD. DNS for `devicenexus.ai` is already at Cloudflare.

**This task is mostly done in the Cloudflare dashboard, not in code. The code changes are minor.**

**Step 1: Add `_headers` file for security headers**

Create `public/_headers`:

```
/*
  X-Frame-Options: DENY
  X-Content-Type-Options: nosniff
  Referrer-Policy: strict-origin-when-cross-origin
```

**Step 2: Connect repo to Cloudflare Pages**

1. Go to Cloudflare Dashboard → Workers & Pages → Create application → Pages
2. Connect to Git → select `devicenexus-docs` repo
3. Build settings:
   - **Build command:** `npm run build`
   - **Build output directory:** `dist`
   - **Node.js version:** `20`
4. Environment variables (if PostHog is added later):
   - `PUBLIC_POSTHOG_KEY` = your key
   - `PUBLIC_POSTHOG_HOST` = `https://us.i.posthog.com`
5. Save and deploy

**Step 3: Add custom domain in Cloudflare Pages**

1. In the Pages project → Custom domains → Add domain
2. Enter `learn.devicenexus.ai`
3. Cloudflare auto-creates the DNS CNAME record (since DNS is already at Cloudflare)

**Step 4: Verify submodule initializes on CF build**

Cloudflare Pages automatically uses `--recurse-submodules` when cloning. Verify by checking the first build log — you should see the submodule being cloned.

If build fails with "submodules/robotics-glossary not found": in CF Pages settings → set the git clone command to include submodules, or add to build command:

```
git submodule update --init && npm run build
```

**Step 5: Verify live site**

After deployment completes:
- `https://learn.devicenexus.ai/` — homepage with 4 panels
- `https://learn.devicenexus.ai/glossary/concepts/fundamentals/kinematics/` — glossary page with sidebar
- `https://learn.devicenexus.ai/guides/act-training-lessons/` — guide with no sidebar, byline
- `https://learn.devicenexus.ai/dispatch/` — dispatch with subscribe form
- `https://learn.devicenexus.ai/dispatch/feed.xml` — valid RSS XML

**Step 6: Commit headers file**

```bash
git add public/_headers
git commit -m "chore: add Cloudflare Pages security headers"
```

---

## Summary of All Tasks

| # | Phase | Repo | Outcome |
|---|-------|------|---------|
| 1 | Cleanup | robotics-glossary | Guides removed, links updated, math plugins removed |
| 2 | Structure | devicenexus-docs | blog→guides, research→papers |
| 3 | Submodule | devicenexus-docs | Glossary synced at build time |
| 4 | Content | devicenexus-docs | Dispatch section + first issue |
| 5 | Navigation | devicenexus-docs | 4-tab header: Glossary/Guides/Dispatch/Papers |
| 6 | Layout | devicenexus-docs | Article layout + bylines + editUrl:false |
| 7 | RSS | devicenexus-docs | /dispatch/feed.xml live |
| 8 | Email | devicenexus-docs | Buttondown subscribe form |
| 9 | Config | devicenexus-docs | site=learn.devicenexus.ai, 4-panel homepage |
| 10 | Content | devicenexus-docs | About page |
| 11 | Deploy | Cloudflare | learn.devicenexus.ai live |

## Updating the Glossary (ongoing workflow)

When `robotics-glossary` gets new content:

```bash
cd /home/amar/baskd/devicenexus-docs
git submodule update --remote submodules/robotics-glossary
git add submodules/robotics-glossary
git commit -m "chore: update glossary to latest"
```

Cloudflare Pages auto-deploys on push.
