---
description: "Task list for Book Infrastructure & Docusaurus Theme"
---

# Tasks: 005-book-infrastructure

**Input**: Design documents from `specs/005-book-infrastructure/`
**Prerequisites**: plan.md, research.md, data-model.md

**Organization**: Tasks are grouped by user story (mapped from Plan Phases) to enable independent implementation.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel
- **[Story]**: [US1] (Content Pipeline), [US2] (Initial Content)

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Initialize Docusaurus project in root with `npx create-docusaurus@latest`
- [x] T002 Configure `docusaurus.config.js` with metadata and plugins
- [x] T003 Set up GitHub Pages deployment workflow in `.github/workflows/deploy.yml`

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T004 Create `specs/`, `docs/`, `src/` directory structure per plan
- [x] T005 Implement `src/css/custom.css` with Neon Cyber AI theme variables
- [x] T006 Install KaTeX and citation plugins (remark-math, rehype-katex)
- [x] T007 Configure Docusaurus presets for KaTeX in `docusaurus.config.js`
- [x] T008 [P] Add `Orbitron` and `Inter` fonts to `src/css/custom.css`

**Checkpoint**: Foundation ready - theme and structure in place.

## Phase 3: User Story 1 - Content Pipeline (Priority: P1) 🎯 MVP

**Goal**: As an author, I want templates and validation scripts so that I can write spec-compliant chapters.

**Independent Test**: Create a dummy chapter using the template and verify it passes the validation script.

### Implementation for User Story 1

- [x] T009 [US1] Create `scripts/verify-specs.js` to check `spec_ref` in docs
- [x] T010 [P] [US1] Create `src/components/ChapterTemplate.md`
- [x] T011 [P] [US1] Create `src/components/ExerciseTemplate.md`
- [x] T012 [US1] Create `.github/workflows/validate-spec.yml` to run verification on PR
- [x] T013 [US1] Add `npm run verify-specs` command to `package.json`

**Checkpoint**: Pipeline ready. Authors can write content with validation.

## Phase 4: User Story 2 - Initial Content (Priority: P2)

**Goal**: As a reader, I want to see the first module draft so that I can verify the book's structure.

**Independent Test**: Build the site and navigate to Module 1; verify "Neon" theme and KaTeX rendering.

### Implementation for User Story 2

- [x] T014 [US2] Create `docs/01-nervous-system/` directory structure
- [x] T015 [US2] Write `docs/01-nervous-system/00-intro.md` with spec reference
- [x] T016 [US2] Add math equation example in intro to test KaTeX
- [x] T017 [US2] Update `sidebars.js` to include Module 1

**Checkpoint**: First content module visible and themed.

## Phase N: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T018 Check color contrast ratios for Neon theme accessibility
- [x] T019 Finalize `README.md` with contribution guidelines

## Dependencies & Execution Order

### Phase Dependencies

- **Setup**: No dependencies.
- **Foundational**: Depends on Setup. Blocks US1, US2.
- **US1 (Pipeline)**: Depends on Foundational.
- **US2 (Content)**: Depends on Foundational (and ideally US1 for validation, but can start parallel).

### Parallel Opportunities

- T005 (CSS) and T006 (Plugins) can run in parallel.
- T010 (Templates) and T009 (Script) can run in parallel.
- US1 and US2 can theoretically run in parallel if US2 authors manually check specs until US1 is ready.

## Implementation Strategy

### MVP First (US1)

1. Complete Setup & Foundational.
2. Implement Content Pipeline (US1).
3. Validate with dummy data.

### Incremental Delivery

1. Foundation (Theme/Config).
2. Pipeline (Validation).
3. Content (Module 1).
