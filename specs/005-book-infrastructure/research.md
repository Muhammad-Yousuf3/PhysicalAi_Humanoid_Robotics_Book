# Research: Book Infrastructure & Theme

**Feature**: 005-book-infrastructure
**Date**: 2025-12-08

## 1. Docusaurus Plugins for Scientific Writing

### Requirement
The book requires academic rigor, including APA-style citations and mathematical formulas for robotics (kinematics/dynamics).

### Findings
-   **Math Support**: Docusaurus officially supports **KaTeX**.
    -   Usage: `npm install --save remark-math rehype-katex`
    -   Config: Add to `presets` in `docusaurus.config.js`.
-   **Citations**:
    -   **Option A**: `rehype-citation`. Parses BibTeX files and renders citations.
    -   **Recommendation**: Use standard Markdown footnotes `[^1]` for simplicity in initial phase, upgrade to `rehype-citation` if bibliography grows >50 items.
    -   *Decision*: Start with **standard MDX footnotes** for portability; evaluate `rehype-citation` for Module 3 (Deep Learning citations).

## 2. Neon Cyber AI Theme Design

### Aesthetic Goals
-   "High-Tech", "Hacker", "Futuristic".
-   Dark Mode by default (force dark mode or make light mode "Cyberpunk Day").
-   High contrast for code readability.

### Color Palette (CSS Variables)

```css
:root {
  /* Base */
  --ifm-color-primary: #00ff41;        /* Matrix Green */
  --ifm-color-primary-dark: #00e63a;
  --ifm-color-primary-darker: #00d937;
  --ifm-color-primary-darkest: #00b32d;
  --ifm-color-primary-light: #1aff54;
  --ifm-color-primary-lighter: #33ff67;
  --ifm-color-primary-lightest: #80ff9f;

  /* Backgrounds */
  --ifm-background-color: #050505;     /* Deep Black */
  --ifm-background-surface-color: #121212; /* Card BG */

  /* Accents */
  --cyber-blue: #00f3ff;
  --cyber-pink: #ff00ff;
  --cyber-purple: #bd00ff;
}
```

### Typography
-   **Headings**: `Orbitron` or `Rajdhani` (Google Fonts) for futuristic feel.
-   **Body**: `Inter` or `Roboto` for readability.
-   **Code**: `Fira Code` (with ligatures).

## 3. GitHub Actions Automation

### Workflow: `validate-spec.yml`
-   **Trigger**: PR to `master`.
-   **Job**:
    1.  Checkout code.
    2.  Run `npm install`.
    3.  Run `npm run build` (Docusaurus build).
    4.  Run custom script `scripts/verify-specs.js` (checks if every MD file in `docs/` has a `spec_ref` metadata).

### Decision Record
-   **Hosting**: GitHub Pages (User Pages) vs Vercel.
    -   *Decision*: **GitHub Pages**. Keeps everything (source + deploy) in one ecosystem. Cost-effective and aligns with the open-source nature.
