# Physical AI & Humanoid Robotics: The Book

An AI-Native, Spec-Driven technical book on building humanoid robots with ROS 2, Isaac Sim, and VLA models.

## Development

This project uses [Docusaurus](https://docusaurus.io/).

### Prerequisites

- Node.js 18+

### Setup

```bash
npm install
```

### Run Locally

```bash
npm start
```
Opens http://localhost:3000

### Validation

To verify that all chapters reference a valid spec:

```bash
npm run verify-specs
```

## Contribution Guidelines

1. **Spec-First**: Create or update a spec in `specs/` before writing content.
2. **Drafting**: Use the templates in `src/components/`.
3. **Validation**: Ensure `npm run verify-specs` passes.

## Directory Structure

- `docs/`: Book chapters (Markdown/MDX).
- `specs/`: Technical specifications.
- `src/`: Custom components and theme.
- `static/`: Images and assets.