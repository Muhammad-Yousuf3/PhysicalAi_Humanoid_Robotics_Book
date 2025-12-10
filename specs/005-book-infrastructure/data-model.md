# Data Model: Book Content

**Feature**: 005-book-infrastructure

## Entities

### 1. Chapter (Markdown/MDX)
Represents a single page of content in the book.

**File Path**: `docs/<module>/<slug>.md`

**Frontmatter Schema**:
```yaml
---
id: <string>           # Unique ID for URL slug
title: <string>        # Display title
sidebar_position: <int> # Order in the sidebar
description: <string>  # SEO description
tags: [<string>]       # Keywords
spec_ref: <path>       # MANDATORY: Path to the spec defining this chapter
status: <enum>         # draft | review | stable
last_update:
  date: <date>
  author: <string>
---
```

### 2. Specification (Markdown)
The source of truth for a feature or chapter set.

**File Path**: `specs/<feature-branch>/spec.md`

**Frontmatter Schema**:
```yaml
---
title: <string>
feature_branch: <string>
status: <enum>         # draft | approved | implemented
---
```

### 3. Asset (Image/Model)
Binary files used in chapters.

**File Path**: `static/img/<module>/<filename>` or `static/models/<filename>`

**Conventions**:
-   Images: `.png`, `.jpg`, `.svg`
-   3D Models: `.glb`, `.usd`, `.urdf`
-   Naming: `kebab-case`

## Relationships

-   **One Spec** → **Many Chapters** (1:N)
    -   A spec defines a Module (e.g., Module 1), which contains multiple chapters.
-   **One Chapter** → **Many Assets** (1:N)

## Validation Rules

1.  Every `.md` file in `docs/` MUST have a `spec_ref` field.
2.  The `spec_ref` path MUST resolve to an existing file in `specs/`.
3.  `id` MUST be unique across the site.
