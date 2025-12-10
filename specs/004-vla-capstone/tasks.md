# Tasks: Module 4 (Vision-Language-Action)

**Feature**: `004-vla-capstone`
**Status**: Pending
**Branch**: `004-vla-capstone`

## Phase 1: Setup
*Goal: Initialize the chapter directory structure and assets.*

- [x] T001 Create documentation directory structure `docs/004-vla-capstone/`
- [x] T002 Create static image directory `static/img/004-vla-capstone/`
- [x] T003 Create diagram tracking file `static/img/004-vla-capstone/README.md` with list of required diagrams

## Phase 2: Foundational
*Goal: Establish the chapter scope, objectives, and introductory concepts (Section 4.1).*

- [x] T004 Create `docs/004-vla-capstone/00-intro.md` with strict frontmatter (title, slug)
- [x] T005 Write "Header" section in `00-intro.md` (Title, Abstract) matching `contracts/chapter-structure.md`
- [x] T006 Write "Learning Objectives" section in `00-intro.md` (3+ objectives including VLA loop, Grounding, Safety)
- [x] T007 Write "Section 4.1: Introduction" in `00-intro.md` explaining the evolution from Chatbots to Robot Brains

## Phase 3: Voice Command & Interpretation (User Story 1)
*Goal: Explain Voice-to-Action and the Whisper pipeline (Section 4.2).*

- [x] T008 [US1] Create `docs/004-vla-capstone/01-voice-to-action.md`
- [x] T009 [US1] Write "Section 4.2: Voice-to-Action" covering the "Ear" (Whisper) and "Mouth" (TTS)
- [x] T010 [US1] Explain the data flow: Audio Capture -> Whisper Inference -> Text Topic (FR-002)
- [x] T011 [P] [US1] Add placeholder description for "VLA Architecture Diagram (Audio focus)" in `01-voice-to-action.md`

## Phase 4: Cognitive Task Planning (User Story 2)
*Goal: Explain LLM-based planning, prompt engineering, and safety (Section 4.3).*

- [x] T012 [US2] Create `docs/004-vla-capstone/02-cognitive-planning.md`
- [x] T013 [US2] Write "Section 4.3: Cognitive Planning" covering the "Prefrontal Cortex" (LLM)
- [x] T014 [US2] Explain "Prompt Engineering" for robotics (System Prompts, In-Context Learning) (FR-003)
- [x] T015 [US2] Define "Action Primitives" and "Code as Policies" concepts (Action Vocabulary)
- [x] T016 [US2] Write "Safety Note": Risks of Hallucination and Safety Filters (FR-004)
- [x] T017 [P] [US2] Add placeholder description for "Prompt Engineering Example Diagram" in `02-cognitive-planning.md`

## Phase 5: Integrated Perception for Planning (User Story 3)
*Goal: Explain Grounding and the Scene Graph (Section 4.4).*

- [x] T018 [US3] Create `docs/004-vla-capstone/03-scene-graph.md`
- [x] T019 [US3] Write "Section 4.4: The Scene Graph" explaining how Vision connects to Language (Grounding)
- [x] T020 [US3] Define the JSON schema for the Scene Graph (Objects, Relations, Attributes) (FR-005)
- [x] T021 [P] [US3] Add placeholder description for "Scene Graph Visualization" in `03-scene-graph.md`

## Phase 6: Autonomous Action Execution (User Story 4)
*Goal: Walkthrough of the Capstone Project and Integration (Section 4.5).*

- [x] T022 [US4] Create `docs/004-vla-capstone/04-capstone-project.md`
- [x] T023 [US4] Write "Section 4.5: Capstone Project" detailing the "Autonomous Tidying Robot" scenario
- [x] T024 [US4] Provide a step-by-step execution trace: Voice -> Text -> Plan -> Action (FR-007)
- [x] T025 [US4] Explain the "Sim-to-Real" safety gap for VLA systems (FR-006)
- [x] T026 [P] [US4] Add "Hallucination Warning" callout in `04-capstone-project.md`

## Phase 7: Polish & Review
*Goal: Finalize content, add review elements, and ensure quality.*

- [x] T027 Create `docs/004-vla-capstone/05-summary.md`
- [x] T028 Write "Key Takeaways", "Future Outlook", and "Quiz" in `05-summary.md`
- [x] T029 Review all files to ensure NO executable hardware code is present (Safety Check)
- [x] T030 Update `sidebars.ts` to include the new Chapter 4 structure

## Dependencies

1.  **Phase 1 & 2** (Setup/Foundation) must be done first.
2.  **Phase 3** (Voice) and **Phase 5** (Perception) provide inputs to **Phase 4** (Planning), but can be drafted in parallel.
3.  **Phase 6** (Capstone) integrates concepts from Phases 3, 4, and 5 and should be written after them.
4.  **Phase 7** (Polish) must happen last.

## Parallel Execution Examples

*   **Author A**: Phase 3 (Voice/Whisper)
*   **Author B**: Phase 4 (Planning/LLM)
*   **Author C**: Phase 5 (Perception/Scene Graph)

## Implementation Strategy

*   **MVP**: Complete Phases 1, 2, and 4 (Core Planning Concepts).
*   **Full Feature**: Complete all Phases including Capstone Walkthrough.
