---
id: 0004
title: Generate Tasks for Digital Twin (Gazebo & Unity)
stage: tasks
date: 2025-12-09
surface: agent
model: gemini-1.5-flash
feature: 002-digital-twin-sim
branch: main
user: muhammad-yousuf
command: ---
description: Generate an actionable, dependency-ordered tasks.md for the feature based on available design artifacts.
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

1. **Setup**: Run `.specify/scripts/bash/check-prerequisites.sh --json` from repo root and parse FEATURE_DIR and AVAILABLE_DOCS list. All paths must be absolute. For single quotes in args like "I'm Groot", use escape syntax: e.g 'I'\'m Groot' (or double-quote if possible: "I'm Groot").

2. **Load design documents**: Read from FEATURE_DIR:
   - **Required**: plan.md (tech stack, libraries, structure), spec.md (user stories with priorities)
   - **Optional**: data-model.md (entities), contracts/ (API endpoints), research.md (decisions), quickstart.md (test scenarios)
   - Note: Not all projects have all documents. Generate tasks based on what's available.

3. **Execute task generation workflow**:
   - Load plan.md and extract tech stack, libraries, project structure
   - Load spec.md and extract user stories with their priorities (P1, P2, P3, etc.)
   - If data-model.md exists: Extract entities and map to user stories
   - If contracts/ exists: Map endpoints to user stories
   - If research.md exists: Extract decisions for setup tasks
   - Generate tasks organized by user story (see Task Generation Rules below)
   - Generate dependency graph showing user story completion order
   - Create parallel execution examples per user story
   - Validate task completeness (each user story has all needed tasks, independently testable)

4. **Generate tasks.md**: Use `.specify.specify/templates/tasks-template.md` as structure, fill with:
   - Correct feature name from plan.md
   - Phase 1: Setup tasks (project initialization)
   - Phase 2: Foundational tasks (blocking prerequisites for all user stories)
   - Phase 3+: One phase per user story (in priority order from spec.md)
   - Each phase includes: story goal, independent test criteria, tests (if requested), implementation tasks
   - Final Phase: Polish & cross-cutting concerns
   - All tasks must follow the strict checklist format (see Task Generation Rules below)
   - Clear file paths for each task
   - Dependencies section showing story completion order
   - Parallel execution examples per user story
   - Implementation strategy section (MVP first, incremental delivery)

5. **Report**: Output path to generated tasks.md and summary:
   - Total task count
   - Task count per user story
   - Parallel opportunities identified
   - Independent test criteria for each story
   - Suggested MVP scope (typically just User Story 1)
   - Format validation: Confirm ALL tasks follow the checklist format (checkbox, ID, labels, file paths)

Context for task generation: `002-digital-twin-sim`,/sp.plan Create an end-to-end project plan for **Module 2: The Digital Twin (Gazebo & Unity)** of the AI_Native_Spec_Driven_Book. This module covers: - Physics-based simulation using Gazebo - High-fidelity environment rendering in Unity - Sensor simulation: LiDAR, depth cameras, IMUs - Digital twin workflows for humanoid robotics Plan must follow the Constitution + specs + template. 
------------------------------------------------------------ 
1. ARCHITECTURE SKETCH 
------------------------------------------------------------ 
Provide a conceptual architecture illustrating:
 - Dual-engine digital twin pipeline:
 - **Gazebo** for physics, collisions, gravity, actuator dynamics
 - **Unity** for high-fidelity visuals, human-robot interaction, cinematic rendering
 - How the humanoid robot URDF flows into Gazebo simulation
 - How Unity imports models for high-detail rendering (glTF/Fbx pipeline)
 - Message interfaces:
 - ROS → Gazebo (control, sensors)
 - ROS → Unity (via ROS–TCP–Connector or custom bridge)
 - Sensor simulation architecture:
 - LiDAR → Raycasting engine outputs
 - Depth camera → GPU render pipeline
 - IMU → noise model + physics integration
 - Folder & file structure for the module inside Docusaurus:
 - /module2/
 - /module2/gazebo/
 - /module2/unity/
 - /module2/sensors/
 - /module2/assets/
Deliver:
 - Block diagram of two-engine simulation
 - Data-flow graph for sensors → ROS topics
 - Feedback loop between simulation and agent controllers 
------------------------------------------------------------ 
2. SECTION STRUCTURE 
------------------------------------------------------------ 
Define the complete chapter outline:
 **Section 1 — Introduction**
 - What is a Digital Twin?
 - Why humanoid robotics require dual simulation engines 
 **Section 2 — Gazebo Physics Simulation**
 - Gazebo classic vs Ignition (Garden/Fortress)
 - Physics engines (ODE, Bullet, DART)
 - Gravity, friction, collisions
 - Actuator + joint dynamics
 - Running humanoid URDF in Gazebo
 - ROS 2 integration (gazebo_ros_pkgs) 
 **Section 3 — Unity for High-Definition Rendering**
 - Unity HDRP/URP rendering pipelines
 - Human-in-the-loop interaction environments
 - Lighting, materials, real-time shadows
 - Unity–ROS 2 bridge using ROS–TCP-Connector
 - Importing humanoid models (FBX, glTF) 
 **Section 4 — Sensor Simulation**
 - LiDAR: raycasting, point cloud generation, noise models
 - Depth camera: GPU depth shader, projection matrices
 - IMU: accelerometer, gyroscope, orientation noise
 - Connecting each sensor to ROS 2 topics 
 **Section 5 — Combined Digital Twin Workflow**
 - Side-by-side physics + rendering pipeline
 - Synchronization challenges
 - Real-world testing vs simulation comparison
 - Performance optimization 
 **Section 6 — Mini Example**
 - Build a small humanoid scene:
 - Gazebo physics playground
 - Unity rendering arena
 - Simulate LiDAR + depth + IMU
 - Export data for agent training 
 **Section 7 — APA References**
Also include templates for:
 - Simulation config files
 - Scene diagrams
 - Sensor pipeline examples
 - Agent–simulation loop examples 
------------------------------------------------------------ 
3. RESEARCH APPROACH 
------------------------------------------------------------ 
Use a **research-concurrent** workflow:
 - Research simulation physics while writing content
 - Verify Unity HDRP/URP and Gazebo physics references (official docs)
 - Validate sensor simulation formulas using peer-reviewed robotics papers
 - Check LiDAR noise models against academic sources
 - Align all descriptions with APA citation rules
Research tasks:
 - Compare physics engines (ODE, Bullet, DART)
 - Simulation resolution tradeoffs
 - Unity’s render pipeline documentation
 - ROS–Unity bridge architecture
 - Sensor modeling from robotics textbooks 
------------------------------------------------------------ 
4. QUALITY VALIDATION 
------------------------------------------------------------ 
Define quality control checks:
 **Simulation correctness**
 - Gazebo physics accuracy validated with official examples
 - Unity visuals correctly mapped to robot transforms
 - Sensor outputs match real-world expectations 
 **Structural consistency**
 - All sensor examples publish to correct ROS 2 topic types
 - Digital twin workflow logically explained
 - Code samples follow reproducibility rules 
 **Visual standards**
 - Diagrams follow Neon Cyber AI theme
 - Clear separation of physics vs rendering engine diagrams 
 **Citation standards**
 - APA compliance
 - Minimum academic sources for sensor noise models 
 **Content validation**
 - All claims measurable, verifiable, and sourced
 - Realistic physics assumptions (gravity, inertia)
 - Sensor values have correct units and ranges 
------------------------------------------------------------ 
5. DECISIONS NEEDING DOCUMENTATION 
------------------------------------------------------------ 
Document decision points with options/tradeoffs:
 - Gazebo version (Classic vs Ignition)
 - Stability vs new features
 - Physics engine choice (ODE vs Bullet vs DART)
 - Speed vs accuracy vs complexity
 - Unity pipeline (URP vs HDRP)
 - Performance vs realism
 - ROS–Unity communication method
 - ROS–TCP–Connector vs custom bridge
 - Sensor fidelity level
 - High accuracy vs real-time performance
 - File format for humanoid import (FBX vs glTF)
 - Compatibility vs detail
Each decision must list:
 - Available options
 - Pros/cons
 - Reason for recommended choice 
------------------------------------------------------------ 
6. TESTING STRATEGY 
------------------------------------------------------------ 
Apply multi-layer validation:
 **Physics testing**
 - Verify robot stands without exploding (stability test)
 - Check collision meshes behave correctly
 - Validate gravity and friction settings 
 **Unity rendering tests**
 - Verify lighting, shadows, materials render correctly
 - Test ROS data visualization in Unity 
 **Sensor testing**
 - LiDAR: 360° scan shape validation
 - Depth camera: correct depth encoding
 - IMU: stable orientation noise model 
 **Agent → Simulation loop tests**
 - Ensure robots receive controller commands from ROS
 - Validate sensor feedback loop timing 
 **APA & formatting tests**
 - Citation format verified
 - Diagrams conform to book visual theme 
------------------------------------------------------------ 
7. ORGANIZE BY PHASES 
------------------------------------------------------------ 
PHASE 1: RESEARCH
 - Study Gazebo physics engines
 - Review Unity rendering pipelines
 - Collect sensor modeling academic references 
PHASE 2: FOUNDATION
 - Build Docusaurus module folder
 - Create base Gazebo world + Unity scene
 - Prepare Neon Cyber AI–style diagrams
 - Adapt the humanoid URDF from Module 1 for Gazebo, adding necessary plugins for physics and sensors 
PHASE 3: ANALYSIS
 - Compare physics & rendering tradeoffs
 - Evaluate communication bridges
 - Validate sensor simulation details
 - Document all architectural decisions 
PHASE 4: SYNTHESIS
 - Write full chapter content
 - Produce runnable sensor simulation examples
 - Generate combined digital twin architecture diagrams
 - Final APA citation review & formatting

The tasks.md should be immediately executable - each task must be specific enough that an LLM can complete it without additional context.

## Task Generation Rules

**CRITICAL**: Tasks MUST be organized by user story to enable independent implementation and testing.

**Tests are OPTIONAL**: Only generate test tasks if explicitly requested in the feature specification or if user requests TDD approach.

### Checklist Format (REQUIRED)

Every task MUST strictly follow this format:

```text
- [ ] [TaskID] [P?] [Story?] Description with file path
```

**Format Components**:

1. **Checkbox**: ALWAYS start with `- [ ]` (markdown checkbox)
2. **Task ID**: Sequential number (T001, T002, T003...) in execution order
3. **[P] marker**: Include ONLY if task is parallelizable (different files, no dependencies on incomplete tasks)
4. **[Story] label**: REQUIRED for user story phase tasks only
   - Format: [US1], [US2], [US3], etc. (maps to user stories from spec.md)
   - Setup phase: NO story label
   - Foundational phase: NO story label  
   - User Story phases: MUST have story label
   - Polish phase: NO story label
5. **Description**: Clear action with exact file path

**Examples**:

- ✅ CORRECT: `- [ ] T001 Create project structure per implementation plan`
- ✅ CORRECT: `- [ ] T005 [P] Implement authentication middleware in src/middleware/auth.py`
- ✅ CORRECT: `- [ ] T012 [P] [US1] Create User model in src/models/user.py`
- ✅ CORRECT: `- [ ] T014 [US1] Implement UserService in src/services/user_service.py`
- ❌ WRONG: `- [ ] Create User model` (missing ID and Story label)
- ❌ WRONG: `T001 [US1] Create model` (missing checkbox)
- ❌ WRONG: `- [ ] [US1] Create User model` (missing Task ID)
- ❌ WRONG: `- [ ] T001 [US1] Create model` (missing file path)

### Task Organization

1. **From User Stories (spec.md)** - PRIMARY ORGANIZATION:
   - Each user story (P1, P2, P3...) gets its own phase
   - Map all related components to their story:
     - Models needed for that story
     - Services needed for that story
     - Endpoints/UI needed for that story
     - If tests requested: Tests specific to that story
   - Mark story dependencies (most stories should be independent)

2. **From Contracts**:
   - Map each contract/endpoint → to the user story it serves
   - If tests requested: Each contract → contract test task [P] before implementation in that story's phase

3. **From Data Model**:
   - Map each entity to the user story(ies) that need it
   - If entity serves multiple stories: Put in earliest story or Setup phase
   - Relationships → service layer tasks in appropriate story phase

4. **From Setup/Infrastructure**:
   - Shared infrastructure → Setup phase (Phase 1)
   - Foundational/blocking tasks → Foundational phase (Phase 2)
   - Story-specific setup → within that story's phase

### Phase Structure

- **Phase 1**: Setup (project initialization)
- **Phase 2**: Foundational (blocking prerequisites - MUST complete before user stories)
- **Phase 3+**: User Stories in priority order (P1, P2, P3...)
  - Within each story: Tests (if requested) → Models → Services → Endpoints → Integration
  - Each phase should be a complete, independently testable increment
- **Final Phase**: Polish & Cross-Cutting Concerns

---

As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

1) Determine Stage
   - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate Title and Determine Routing:
   - Generate Title: 3–7 words (slug for filename)
   - Route is automatically determined by stage:
     - `constitution` → `history/prompts/constitution/`
     - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)
     - `general` → `history/prompts/general/`

3) Create and Fill PHR (Shell first; fallback agent‑native)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.
   - If the script fails:
     - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)
     - Allocate an ID; compute the output path based on stage from step 2; write the file
     - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

4) Validate + report
   - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
   - On failure: warn, don't block. Skip only for `/sp.phr`.
---

## Prompt

---
description: Generate an actionable, dependency-ordered tasks.md for the feature based on available design artifacts.
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

1. **Setup**: Run `.specify/scripts/bash/check-prerequisites.sh --json` from repo root and parse FEATURE_DIR and AVAILABLE_DOCS list. All paths must be absolute. For single quotes in args like "I'm Groot", use escape syntax: e.g 'I'\'m Groot' (or double-quote if possible: "I'm Groot").

2. **Load design documents**: Read from FEATURE_DIR:
   - **Required**: plan.md (tech stack, libraries, structure), spec.md (user stories with priorities)
   - **Optional**: data-model.md (entities), contracts/ (API endpoints), research.md (decisions), quickstart.md (test scenarios)
   - Note: Not all projects have all documents. Generate tasks based on what's available.

3. **Execute task generation workflow**:
   - Load plan.md and extract tech stack, libraries, project structure
   - Load spec.md and extract user stories with their priorities (P1, P2, P3, etc.)
   - If data-model.md exists: Extract entities and map to user stories
   - If contracts/ exists: Map endpoints to user stories
   - If research.md exists: Extract decisions for setup tasks
   - Generate tasks organized by user story (see Task Generation Rules below)
   - Generate dependency graph showing user story completion order
   - Create parallel execution examples per user story
   - Validate task completeness (each user story has all needed tasks, independently testable)

4. **Generate tasks.md**: Use `.specify.specify/templates/tasks-template.md` as structure, fill with:
   - Correct feature name from plan.md
   - Phase 1: Setup tasks (project initialization)
   - Phase 2: Foundational tasks (blocking prerequisites for all user stories)
   - Phase 3+: One phase per user story (in priority order from spec.md)
   - Each phase includes: story goal, independent test criteria, tests (if requested), implementation tasks
   - Final Phase: Polish & cross-cutting concerns
   - All tasks must follow the strict checklist format (see Task Generation Rules below)
   - Clear file paths for each task
   - Dependencies section showing story completion order
   - Parallel execution examples per user story
   - Implementation strategy section (MVP first, incremental delivery)

5. **Report**: Output path to generated tasks.md and summary:
   - Total task count
   - Task count per user story
   - Parallel opportunities identified
   - Independent test criteria for each story
   - Suggested MVP scope (typically just User Story 1)
   - Format validation: Confirm ALL tasks follow the checklist format (checkbox, ID, labels, file paths)

Context for task generation: `002-digital-twin-sim`,/sp.plan Create an end-to-end project plan for **Module 2: The Digital Twin (Gazebo & Unity)** of the AI_Native_Spec_Driven_Book. This module covers: - Physics-based simulation using Gazebo - High-fidelity environment rendering in Unity - Sensor simulation: LiDAR, depth cameras, IMUs - Digital twin workflows for humanoid robotics Plan must follow the Constitution + specs + template. 
------------------------------------------------------------ 
1. ARCHITECTURE SKETCH 
------------------------------------------------------------ 
Provide a conceptual architecture illustrating:
 - Dual-engine digital twin pipeline:
 - **Gazebo** for physics, collisions, gravity, actuator dynamics
 - **Unity** for high-fidelity visuals, human-robot interaction, cinematic rendering
 - How the humanoid robot URDF flows into Gazebo simulation
 - How Unity imports models for high-detail rendering (glTF/Fbx pipeline)
 - Message interfaces:
 - ROS → Gazebo (control, sensors)
 - ROS → Unity (via ROS–TCP–Connector or custom bridge)
 - Sensor simulation architecture:
 - LiDAR → Raycasting engine outputs
 - Depth camera → GPU render pipeline
 - IMU → noise model + physics integration
 - Folder & file structure for the module inside Docusaurus:
 - /module2/
 - /module2/gazebo/
 - /module2/unity/
 - /module2/sensors/
 - /module2/assets/
Deliver:
 - Block diagram of two-engine simulation
 - Data-flow graph for sensors → ROS topics
 - Feedback loop between simulation and agent controllers 
------------------------------------------------------------ 
2. SECTION STRUCTURE 
------------------------------------------------------------ 
Define the complete chapter outline:
 **Section 1 — Introduction**
 - What is a Digital Twin?
 - Why humanoid robotics require dual simulation engines 
 **Section 2 — Gazebo Physics Simulation**
 - Gazebo classic vs Ignition (Garden/Fortress)
 - Physics engines (ODE, Bullet, DART)
 - Gravity, friction, collisions
 - Actuator + joint dynamics
 - Running humanoid URDF in Gazebo
 - ROS 2 integration (gazebo_ros_pkgs) 
 **Section 3 — Unity for High-Definition Rendering**
 - Unity HDRP/URP rendering pipelines
 - Human-in-the-loop interaction environments
 - Lighting, materials, real-time shadows
 - Unity–ROS 2 bridge using ROS–TCP-Connector
 - Importing humanoid models (FBX, glTF) 
 **Section 4 — Sensor Simulation**
 - LiDAR: raycasting, point cloud generation, noise models
 - Depth camera: GPU depth shader, projection matrices
 - IMU: accelerometer, gyroscope, orientation noise
 - Connecting each sensor to ROS 2 topics 
 **Section 5 — Combined Digital Twin Workflow**
 - Side-by-side physics + rendering pipeline
 - Synchronization challenges
 - Real-world testing vs simulation comparison
 - Performance optimization 
 **Section 6 — Mini Example**
 - Build a small humanoid scene:
 - Gazebo physics playground
 - Unity rendering arena
 - Simulate LiDAR + depth + IMU
 - Export data for agent training 
 **Section 7 — APA References**
Also include templates for:
 - Simulation config files
 - Scene diagrams
 - Sensor pipeline examples
 - Agent–simulation loop examples 
------------------------------------------------------------ 
3. RESEARCH APPROACH 
------------------------------------------------------------ 
Use a **research-concurrent** workflow:
 - Research simulation physics while writing content
 - Verify Unity HDRP/URP and Gazebo physics references (official docs)
 - Validate sensor simulation formulas using peer-reviewed robotics papers
 - Check LiDAR noise models against academic sources
 - Align all descriptions with APA citation rules
Research tasks:
 - Compare physics engines (ODE, Bullet, DART)
 - Simulation resolution tradeoffs
 - Unity’s render pipeline documentation
 - ROS–Unity bridge architecture
 - Sensor modeling from robotics textbooks 
------------------------------------------------------------ 
4. QUALITY VALIDATION 
------------------------------------------------------------ 
Define quality control checks:
 **Simulation correctness**
 - Gazebo physics accuracy validated with official examples
 - Unity visuals correctly mapped to robot transforms
 - Sensor outputs match real-world expectations 
 **Structural consistency**
 - All sensor examples publish to correct ROS 2 topic types
 - Digital twin workflow logically explained
 - Code samples follow reproducibility rules 
 **Visual standards**
 - Diagrams follow Neon Cyber AI theme
 - Clear separation of physics vs rendering engine diagrams 
 **Citation standards**
 - APA compliance
 - Minimum academic sources for sensor noise models 
 **Content validation**
 - All claims measurable, verifiable, and sourced
 - Realistic physics assumptions (gravity, inertia)
 - Sensor values have correct units and ranges 
------------------------------------------------------------ 
5. DECISIONS NEEDING DOCUMENTATION 
------------------------------------------------------------ 
Document decision points with options/tradeoffs:
 - Gazebo version (Classic vs Ignition)
 - Stability vs new features
 - Physics engine choice (ODE vs Bullet vs DART)
 - Speed vs accuracy vs complexity
 - Unity pipeline (URP vs HDRP)
 - Performance vs realism
 - ROS–Unity communication method
 - ROS–TCP–Connector vs custom bridge
 - Sensor fidelity level
 - High accuracy vs real-time performance
 - File format for humanoid import (FBX vs glTF)
 - Compatibility vs detail
Each decision must list:
 - Available options
 - Pros/cons
 - Reason for recommended choice 
------------------------------------------------------------ 
6. TESTING STRATEGY 
------------------------------------------------------------ 
Apply multi-layer validation:
 **Physics testing**
 - Verify robot stands without exploding (stability test)
 - Check collision meshes behave correctly
 - Validate gravity and friction settings 
 **Unity rendering tests**
 - Verify lighting, shadows, materials render correctly
 - Test ROS data visualization in Unity 
 **Sensor testing**
 - LiDAR: 360° scan shape validation
 - Depth camera: correct depth encoding
 - IMU: stable orientation noise model 
 **Agent → Simulation loop tests**
 - Ensure robots receive controller commands from ROS
 - Validate sensor feedback loop timing 
 **APA & formatting tests**
 - Citation format verified
 - Diagrams conform to book visual theme 
------------------------------------------------------------ 
7. ORGANIZE BY PHASES 
------------------------------------------------------------ 
PHASE 1: RESEARCH
 - Study Gazebo physics engines
 - Review Unity rendering pipelines
 - Collect sensor modeling academic references 
PHASE 2: FOUNDATION
 - Build Docusaurus module folder
 - Create base Gazebo world + Unity scene
 - Prepare Neon Cyber AI–style diagrams
 - Adapt the humanoid URDF from Module 1 for Gazebo, adding necessary plugins for physics and sensors 
PHASE 3: ANALYSIS
 - Compare physics & rendering tradeoffs
 - Evaluate communication bridges
 - Validate sensor simulation details
 - Document all architectural decisions 
PHASE 4: SYNTHESIS
 - Write full chapter content
 - Produce runnable sensor simulation examples
 - Generate combined digital twin architecture diagrams
 - Final APA citation review & formatting

The tasks.md should be immediately executable - each task must be specific enough that an LLM can complete it without additional additional context.

## Task Generation Rules

**CRITICAL**: Tasks MUST be organized by user story to enable independent implementation and testing.

**Tests are OPTIONAL**: Only generate test tasks if explicitly requested in the feature specification or if user requests TDD approach.

### Checklist Format (REQUIRED)

Every task MUST strictly follow this format:

```text
- [ ] [TaskID] [P?] [Story?] Description with file path
```

**Format Components**:

1. **Checkbox**: ALWAYS start with `- [ ]` (markdown checkbox)
2. **Task ID**: Sequential number (T001, T002, T003...) in execution order
3. **[P] marker**: Include ONLY if task is parallelizable (different files, no dependencies on incomplete tasks)
4. **[Story] label**: REQUIRED for user story phase tasks only
   - Format: [US1], [US2], [US3], etc. (maps to user stories from spec.md)
   - Setup phase: NO story label
   - Foundational phase: NO story label  
   - User Story phases: MUST have story label
   - Polish phase: NO story label
5. **Description**: Clear action with exact file path

**Examples**:

- ✅ CORRECT: `- [ ] T001 Create project structure per implementation plan`
- ✅ CORRECT: `- [ ] T005 [P] Implement authentication middleware in src/middleware/auth.py`
- ✅ CORRECT: `- [ ] T012 [P] [US1] Create User model in src/models/user.py`
- ✅ CORRECT: `- [ ] T014 [US1] Implement UserService in src/services/user_service.py`
- ❌ WRONG: `- [ ] Create User model` (missing ID and Story label)
- ❌ WRONG: `T001 [US1] Create model` (missing checkbox)
- ❌ WRONG: `- [ ] [US1] Create User model` (missing Task ID)
- ❌ WRONG: `- [ ] T001 [US1] Create model` (missing file path)

### Task Organization

1. **From User Stories (spec.md)** - PRIMARY ORGANIZATION:
   - Each user story (P1, P2, P3...) gets its own phase
   - Map all related components to their story:
     - Models needed for that story
     - Services needed for that story
     - Endpoints/UI needed for that story
     - If tests requested: Tests specific to that story
   - Mark story dependencies (most stories should be independent)

2. **From Contracts**:
   - Map each contract/endpoint → to the user story it serves
   - If tests requested: Each contract → contract test task [P] before implementation in that story's phase

3. **From Data Model**:
   - Map each entity to the user story(ies) that need it
   - If entity serves multiple stories: Put in earliest story or Setup phase
   - Relationships → service layer tasks in appropriate story phase

4. **From Setup/Infrastructure**:
   - Shared infrastructure → Setup phase (Phase 1)
   - Foundational/blocking tasks → Foundational phase (Phase 2)
   - Story-specific setup → within that story's phase

### Phase Structure

- **Phase 1**: Setup (project initialization)
- **Phase 2**: Foundational (blocking prerequisites - MUST complete before user stories)
- **Phase 3+**: User Stories in priority order (P1, P2, P3...)
  - Within each story: Tests (if requested) → Models → Services → Endpoints → Integration
  - Each phase should be a complete, independently testable increment
- **Final Phase**: Polish & Cross-Cutting Concerns

---

As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

1) Determine Stage
   - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate Title and Determine Routing:
   - Generate Title: 3–7 words (slug for filename)
   - Route is automatically determined by stage:
     - `constitution` → `history/prompts/constitution/`
     - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)
     - `general` → `history/prompts/general/`

3) Create and Fill PHR (Shell first; fallback agent‑native)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.
   - If the script fails:
     - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)
     - Allocate an ID; compute the output path based on stage from step 2; write the file
     - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

4) Validate + report
   - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
   - On failure: warn, don't block. Skip only for `/sp.phr`.
---

## Prompt

---
description: Generate an actionable, dependency-ordered tasks.md for the feature based on available design artifacts.
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

1. **Setup**: Run `.specify/scripts/bash/check-prerequisites.sh --json` from repo root and parse FEATURE_DIR and AVAILABLE_DOCS list. All paths must be absolute. For single quotes in args like "I'm Groot", use escape syntax: e.g 'I'\'m Groot' (or double-quote if possible: "I'm Groot").

2. **Load design documents**: Read from FEATURE_DIR:
   - **Required**: plan.md (tech stack, libraries, structure), spec.md (user stories with priorities)
   - **Optional**: data-model.md (entities), contracts/ (API endpoints), research.md (decisions), quickstart.md (test scenarios)
   - Note: Not all projects have all documents. Generate tasks based on what's available.

3. **Execute task generation workflow**:
   - Load plan.md and extract tech stack, libraries, project structure
   - Load spec.md and extract user stories with their priorities (P1, P2, P3, etc.)
   - If data-model.md exists: Extract entities and map to user stories
   - If contracts/ exists: Map endpoints to user stories
   - If research.md exists: Extract decisions for setup tasks
   - Generate tasks organized by user story (see Task Generation Rules below)
   - Generate dependency graph showing user story completion order
   - Create parallel execution examples per user story
   - Validate task completeness (each user story has all needed tasks, independently testable)

4. **Generate tasks.md**: Use `.specify.specify/templates/tasks-template.md` as structure, fill with:
   - Correct feature name from plan.md
   - Phase 1: Setup tasks (project initialization)
   - Phase 2: Foundational tasks (blocking prerequisites for all user stories)
   - Phase 3+: One phase per user story (in priority order from spec.md)
   - Each phase includes: story goal, independent test criteria, tests (if requested), implementation tasks
   - Final Phase: Polish & cross-cutting concerns
   - All tasks must follow the strict checklist format (see Task Generation Rules below)
   - Clear file paths for each task
   - Dependencies section showing story completion order
   - Parallel execution examples per user story
   - Implementation strategy section (MVP first, incremental delivery)

5. **Report**: Output path to generated tasks.md and summary:
   - Total task count
   - Task count per user story
   - Parallel opportunities identified
   - Independent test criteria for each story
   - Suggested MVP scope (typically just User Story 1)
   - Format validation: Confirm ALL tasks follow the checklist format (checkbox, ID, labels, file paths)

Context for task generation: `002-digital-twin-sim`,/sp.plan Create an end-to-end project plan for **Module 2: The Digital Twin (Gazebo & Unity)** of the AI_Native_Spec_Driven_Book. This module covers: - Physics-based simulation using Gazebo - High-fidelity environment rendering in Unity - Sensor simulation: LiDAR, depth cameras, IMUs - Digital twin workflows for humanoid robotics Plan must follow the Constitution + specs + template. 
------------------------------------------------------------ 
1. ARCHITECTURE SKETCH 
------------------------------------------------------------ 
Provide a conceptual architecture illustrating:
 - Dual-engine digital twin pipeline:
 - **Gazebo** for physics, collisions, gravity, actuator dynamics
 - **Unity** for high-fidelity visuals, human-robot interaction, cinematic rendering
 - How the humanoid robot URDF flows into Gazebo simulation
 - How Unity imports models for high-detail rendering (glTF/Fbx pipeline)
 - Message interfaces:
 - ROS → Gazebo (control, sensors)
 - ROS → Unity (via ROS–TCP–Connector or custom bridge)
 - Sensor simulation architecture:
 - LiDAR → Raycasting engine outputs
 - Depth camera → GPU render pipeline
 - IMU → noise model + physics integration
 - Folder & file structure for the module inside Docusaurus:
 - /module2/
 - /module2/gazebo/
 - /module2/unity/
 - /module2/sensors/
 - /module2/assets/
Deliver:
 - Block diagram of two-engine simulation
 - Data-flow graph for sensors → ROS topics
 - Feedback loop between simulation and agent controllers 
------------------------------------------------------------ 
2. SECTION STRUCTURE 
------------------------------------------------------------ 
Define the complete chapter outline:
 **Section 1 — Introduction**
 - What is a Digital Twin?
 - Why humanoid robotics require dual simulation engines 
 **Section 2 — Gazebo Physics Simulation**
 - Gazebo classic vs Ignition (Garden/Fortress)
 - Physics engines (ODE, Bullet, DART)
 - Gravity, friction, collisions
 - Actuator + joint dynamics
 - Running humanoid URDF in Gazebo
 - ROS 2 integration (gazebo_ros_pkgs) 
 **Section 3 — Unity for High-Definition Rendering**
 - Unity HDRP/URP rendering pipelines
 - Human-in-the-loop interaction environments
 - Lighting, materials, real-time shadows
 - Unity–ROS 2 bridge using ROS–TCP-Connector
 - Importing humanoid models (FBX, glTF) 
 **Section 4 — Sensor Simulation**
 - LiDAR: raycasting, point cloud generation, noise models
 - Depth camera: GPU depth shader, projection matrices
 - IMU: accelerometer, gyroscope, orientation noise
 - Connecting each sensor to ROS 2 topics 
 **Section 5 — Combined Digital Twin Workflow**
 - Side-by-side physics + rendering pipeline
 - Synchronization challenges
 - Real-world testing vs simulation comparison
 - Performance optimization 
 **Section 6 — Mini Example**
 - Build a small humanoid scene:
 - Gazebo physics playground
 - Unity rendering arena
 - Simulate LiDAR + depth + IMU
 - Export data for agent training 
 **Section 7 — APA References**
Also include templates for:
 - Simulation config files
 - Scene diagrams
 - Sensor pipeline examples
 - Agent–simulation loop examples 
------------------------------------------------------------ 
3. RESEARCH APPROACH 
------------------------------------------------------------ 
Use a **research-concurrent** workflow:
 - Research simulation physics while writing content
 - Verify Unity HDRP/URP and Gazebo physics references (official docs)
 - Validate sensor simulation formulas using peer-reviewed robotics papers
 - Check LiDAR noise models against academic sources
 - Align all descriptions with APA citation rules
Research tasks:
 - Compare physics engines (ODE, Bullet, DART)
 - Simulation resolution tradeoffs
 - Unity’s render pipeline documentation
 - ROS–Unity bridge architecture
 - Sensor modeling from robotics textbooks 
------------------------------------------------------------ 
4. QUALITY VALIDATION 
------------------------------------------------------------ 
Define quality control checks:
 **Simulation correctness**
 - Gazebo physics accuracy validated with official examples
 - Unity visuals correctly mapped to robot transforms
 - Sensor outputs match real-world expectations 
 **Structural consistency**
 - All sensor examples publish to correct ROS 2 topic types
 - Digital twin workflow logically explained
 - Code samples follow reproducibility rules 
 **Visual standards**
 - Diagrams follow Neon Cyber AI theme
 - Clear separation of physics vs rendering engine diagrams 
 **Citation standards**
 - APA compliance
 - Minimum academic sources for sensor noise models 
 **Content validation**
 - All claims measurable, verifiable, and sourced
 - Realistic physics assumptions (gravity, inertia)
 - Sensor values have correct units and ranges 
------------------------------------------------------------ 
5. DECISIONS NEEDING DOCUMENTATION 
------------------------------------------------------------ 
Document decision points with options/tradeoffs:
 - Gazebo version (Classic vs Ignition)
 - Stability vs new features
 - Physics engine choice (ODE vs Bullet vs DART)
 - Speed vs accuracy vs complexity
 - Unity pipeline (URP vs HDRP)
 - Performance vs realism
 - ROS–Unity communication method
 - ROS–TCP–Connector vs custom bridge
 - Sensor fidelity level
 - High accuracy vs real-time performance
 - File format for humanoid import (FBX vs glTF)
 - Compatibility vs detail
Each decision must list:
 - Available options
 - Pros/cons
 - Reason for recommended choice 
------------------------------------------------------------ 
6. TESTING STRATEGY 
------------------------------------------------------------ 
Apply multi-layer validation:
 **Physics testing**
 - Verify robot stands without exploding (stability test)
 - Check collision meshes behave correctly
 - Validate gravity and friction settings 
 **Unity rendering tests**
 - Verify lighting, shadows, materials render correctly
 - Test ROS data visualization in Unity 
 **Sensor testing**
 - LiDAR: 360° scan shape validation
 - Depth camera: correct depth encoding
 - IMU: stable orientation noise model 
 **Agent → Simulation loop tests**
 - Ensure robots receive controller commands from ROS
 - Validate sensor feedback loop timing 
 **APA & formatting tests**
 - Citation format verified
 - Diagrams conform to book visual theme 
------------------------------------------------------------ 
7. ORGANIZE BY PHASES 
------------------------------------------------------------ 
PHASE 1: RESEARCH
 - Study Gazebo physics engines
 - Review Unity rendering pipelines
 - Collect sensor modeling academic references 
PHASE 2: FOUNDATION
 - Build Docusaurus module folder
 - Create base Gazebo world + Unity scene
 - Prepare Neon Cyber AI–style diagrams
 - Adapt the humanoid URDF from Module 1 for Gazebo, adding necessary plugins for physics and sensors 
PHASE 3: ANALYSIS
 - Compare physics & rendering tradeoffs
 - Evaluate communication bridges
 - Validate sensor simulation details
 - Document all architectural decisions 
PHASE 4: SYNTHESIS
 - Write full chapter content
 - Produce runnable sensor simulation examples
 - Generate combined digital twin architecture diagrams
 - Final APA citation review & formatting

The tasks.md should be immediately executable - each task must be specific enough that an LLM can complete it without additional context.

## Task Generation Rules

**CRITICAL**: Tasks MUST be organized by user story to enable independent implementation and testing.

**Tests are OPTIONAL**: Only generate test tasks if explicitly requested in the feature specification or if user requests TDD approach.

### Checklist Format (REQUIRED)

Every task MUST strictly follow this format:

```text
- [ ] [TaskID] [P?] [Story?] Description with file path
```

**Format Components**:

1. **Checkbox**: ALWAYS start with `- [ ]` (markdown checkbox)
2. **Task ID**: Sequential number (T001, T002, T003...) in execution order
3. **[P] marker**: Include ONLY if task is parallelizable (different files, no dependencies on incomplete tasks)
4. **[Story] label**: REQUIRED for user story phase tasks only
   - Format: [US1], [US2], [US3], etc. (maps to user stories from spec.md)
   - Setup phase: NO story label
   - Foundational phase: NO story label  
   - User Story phases: MUST have story label
   - Polish phase: NO story label
5. **Description**: Clear action with exact file path

**Examples**:

- ✅ CORRECT: `- [ ] T001 Create project structure per implementation plan`
- ✅ CORRECT: `- [ ] T005 [P] Implement authentication middleware in src/middleware/auth.py`
- ✅ CORRECT: `- [ ] T012 [P] [US1] Create User model in src/models/user.py`
- ✅ CORRECT: `- [ ] T014 [US1] Implement UserService in src/services/user_service.py`
- ❌ WRONG: `- [ ] Create User model` (missing ID and Story label)
- ❌ WRONG: `T001 [US1] Create model` (missing checkbox)
- ❌ WRONG: `- [ ] [US1] Create User model` (missing Task ID)
- ❌ WRONG: `- [ ] T001 [US1] Create model` (missing file path)

### Task Organization

1. **From User Stories (spec.md)** - PRIMARY ORGANIZATION:
   - Each user story (P1, P2, P3...) gets its own phase
   - Map all related components to their story:
     - Models needed for that story
     - Services needed for that story
     - Endpoints/UI needed for that story
     - If tests requested: Tests specific to that story
   - Mark story dependencies (most stories should be independent)

2. **From Contracts**:
   - Map each contract/endpoint → to the user story it serves
   - If tests requested: Each contract → contract test task [P] before implementation in that story's phase

3. **From Data Model**:
   - Map each entity to the user story(ies) that need it
   - If entity serves multiple stories: Put in earliest story or Setup phase
   - Relationships → service layer tasks in appropriate story phase

4. **From Setup/Infrastructure**:
   - Shared infrastructure → Setup phase (Phase 1)
   - Foundational/blocking tasks → Foundational phase (Phase 2)
   - Story-specific setup → within that story's phase

### Phase Structure

- **Phase 1**: Setup (project initialization)
- **Phase 2**: Foundational (blocking prerequisites - MUST complete before user stories)
- **Phase 3+**: User Stories in priority order (P1, P2, P3...)
  - Within each story: Tests (if requested) → Models → Services → Endpoints → Integration
  - Each phase should be a complete, independently testable increment
- **Final Phase**: Polish & Cross-Cutting Concerns

---

As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

1) Determine Stage
   - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate Title and Determine Routing:
   - Generate Title: 3–7 words (slug for filename)
   - Route is automatically determined by stage:
     - `constitution` → `history/prompts/constitution/`
     - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)
     - `general` → `history/prompts/general/`

3) Create and Fill PHR (Shell first; fallback agent‑native)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.
   - If the script fails:
     - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)
     - Allocate an ID; compute the output path based on stage from step 2; write the file
     - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

4) Validate + report
   - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
   - On failure: warn, don't block. Skip only for `/sp.phr`.
---

## Prompt

---
description: Generate an actionable, dependency-ordered tasks.md for the feature based on available design artifacts.
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

1. **Setup**: Run `.specify/scripts/bash/check-prerequisites.sh --json` from repo root and parse FEATURE_DIR and AVAILABLE_DOCS list. All paths must be absolute. For single quotes in args like "I'm Groot", use escape syntax: e.g 'I'\'m Groot' (or double-quote if possible: "I'm Groot").

2. **Load design documents**: Read from FEATURE_DIR:
   - **Required**: plan.md (tech stack, libraries, structure), spec.md (user stories with priorities)
   - **Optional**: data-model.md (entities), contracts/ (API endpoints), research.md (decisions), quickstart.md (test scenarios)
   - Note: Not all projects have all documents. Generate tasks based on what's available.

3. **Execute task generation workflow**:
   - Load plan.md and extract tech stack, libraries, project structure
   - Load spec.md and extract user stories with their priorities (P1, P2, P3, etc.)
   - If data-model.md exists: Extract entities and map to user stories
   - If contracts/ exists: Map endpoints to user stories
   - If research.md exists: Extract decisions for setup tasks
   - Generate tasks organized by user story (see Task Generation Rules below)
   - Generate dependency graph showing user story completion order
   - Create parallel execution examples per user story
   - Validate task completeness (each user story has all needed tasks, independently testable)

4. **Generate tasks.md**: Use `.specify.specify/templates/tasks-template.md` as structure, fill with:
   - Correct feature name from plan.md
   - Phase 1: Setup tasks (project initialization)
   - Phase 2: Foundational tasks (blocking prerequisites for all user stories)
   - Phase 3+: One phase per user story (in priority order from spec.md)
   - Each phase includes: story goal, independent test criteria, tests (if requested), implementation tasks
   - Final Phase: Polish & cross-cutting concerns
   - All tasks must follow the strict checklist format (see Task Generation Rules below)
   - Clear file paths for each task
   - Dependencies section showing story completion order
   - Parallel execution examples per user story
   - Implementation strategy section (MVP first, incremental delivery)

5. **Report**: Output path to generated tasks.md and summary:
   - Total task count
   - Task count per user story
   - Parallel opportunities identified
   - Independent test criteria for each story
   - Suggested MVP scope (typically just User Story 1)
   - Format validation: Confirm ALL tasks follow the checklist format (checkbox, ID, labels, file paths)

Context for task generation: `002-digital-twin-sim`,/sp.plan Create an end-to-end project plan for **Module 2: The Digital Twin (Gazebo & Unity)** of the AI_Native_Spec_Driven_Book. This module covers: - Physics-based simulation using Gazebo - High-fidelity environment rendering in Unity - Sensor simulation: LiDAR, depth cameras, IMUs - Digital twin workflows for humanoid robotics Plan must follow the Constitution + specs + template. 
------------------------------------------------------------ 
1. ARCHITECTURE SKETCH 
------------------------------------------------------------ 
Provide a conceptual architecture illustrating:
 - Dual-engine digital twin pipeline:
 - **Gazebo** for physics, collisions, gravity, actuator dynamics
 - **Unity** for high-fidelity visuals, human-robot interaction, cinematic rendering
 - How the humanoid robot URDF flows into Gazebo simulation
 - How Unity imports models for high-detail rendering (glTF/Fbx pipeline)
 - Message interfaces:
 - ROS → Gazebo (control, sensors)
 - ROS → Unity (via ROS–TCP–Connector or custom bridge)
 - Sensor simulation architecture:
 - LiDAR → Raycasting engine outputs
 - Depth camera → GPU render pipeline
 - IMU → noise model + physics integration
 - Folder & file structure for the module inside Docusaurus:
 - /module2/
 - /module2/gazebo/
 - /module2/unity/
 - /module2/sensors/
 - /module2/assets/
Deliver:
 - Block diagram of two-engine simulation
 - Data-flow graph for sensors → ROS topics
 - Feedback loop between simulation and agent controllers 
------------------------------------------------------------ 
2. SECTION STRUCTURE 
------------------------------------------------------------ 
Define the complete chapter outline:
 **Section 1 — Introduction**
 - What is a Digital Twin?
 - Why humanoid robotics require dual simulation engines 
 **Section 2 — Gazebo Physics Simulation**
 - Gazebo classic vs Ignition (Garden/Fortress)
 - Physics engines (ODE, Bullet, DART)
 - Gravity, friction, collisions
 - Actuator + joint dynamics
 - Running humanoid URDF in Gazebo
 - ROS 2 integration (gazebo_ros_pkgs) 
 **Section 3 — Unity for High-Definition Rendering**
 - Unity HDRP/URP rendering pipelines
 - Human-in-the-loop interaction environments
 - Lighting, materials, real-time shadows
 - Unity–ROS 2 bridge using ROS–TCP-Connector
 - Importing humanoid models (FBX, glTF) 
 **Section 4 — Sensor Simulation**
 - LiDAR: raycasting, point cloud generation, noise models
 - Depth camera: GPU depth shader, projection matrices
 - IMU: accelerometer, gyroscope, orientation noise
 - Connecting each sensor to ROS 2 topics 
 **Section 5 — Combined Digital Twin Workflow**
 - Side-by-side physics + rendering pipeline
 - Synchronization challenges
 - Real-world testing vs simulation comparison
 - Performance optimization 
 **Section 6 — Mini Example**
 - Build a small humanoid scene:
 - Gazebo physics playground
 - Unity rendering arena
 - Simulate LiDAR + depth + IMU
 - Export data for agent training 
 **Section 7 — APA References**
Also include templates for:
 - Simulation config files
 - Scene diagrams
 - Sensor pipeline examples
 - Agent–simulation loop examples 
------------------------------------------------------------ 
3. RESEARCH APPROACH 
------------------------------------------------------------ 
Use a **research-concurrent** workflow:
 - Research simulation physics while writing content
 - Verify Unity HDRP/URP and Gazebo physics references (official docs)
 - Validate sensor simulation formulas using peer-reviewed robotics papers
 - Check LiDAR noise models against academic sources
 - Align all descriptions with APA citation rules
Research tasks:
 - Compare physics engines (ODE, Bullet, DART)
 - Simulation resolution tradeoffs
 - Unity’s render pipeline documentation
 - ROS–Unity bridge architecture
 - Sensor modeling from robotics textbooks 
------------------------------------------------------------ 
4. QUALITY VALIDATION 
------------------------------------------------------------ 
Define quality control checks:
 **Simulation correctness**
 - Gazebo physics accuracy validated with official examples
 - Unity visuals correctly mapped to robot transforms
 - Sensor outputs match real-world expectations 
 **Structural consistency**
 - All sensor examples publish to correct ROS 2 topic types
 - Digital twin workflow logically explained
 - Code samples follow reproducibility rules 
 **Visual standards**
 - Diagrams follow Neon Cyber AI theme
 - Clear separation of physics vs rendering engine diagrams 
 **Citation standards**
 - APA compliance
 - Minimum academic sources for sensor noise models 
 **Content validation**
 - All claims measurable, verifiable, and sourced
 - Realistic physics assumptions (gravity, inertia)
 - Sensor values have correct units and ranges 
------------------------------------------------------------ 
5. DECISIONS NEEDING DOCUMENTATION 
------------------------------------------------------------ 
Document decision points with options/tradeoffs:
 - Gazebo version (Classic vs Ignition)
 - Stability vs new features
 - Physics engine choice (ODE vs Bullet vs DART)
 - Speed vs accuracy vs complexity
 - Unity pipeline (URP vs HDRP)
 - Performance vs realism
 - ROS–Unity communication method
 - ROS–TCP–Connector vs custom bridge
 - Sensor fidelity level
 - High accuracy vs real-time performance
 - File format for humanoid import (FBX vs glTF)
 - Compatibility vs detail
Each decision must list:
 - Available options
 - Pros/cons
 - Reason for recommended choice 
------------------------------------------------------------ 
6. TESTING STRATEGY 
------------------------------------------------------------ 
Apply multi-layer validation:
 **Physics testing**
 - Verify robot stands without exploding (stability test)
 - Check collision meshes behave correctly
 - Validate gravity and friction settings 
 **Unity rendering tests**
 - Verify lighting, shadows, materials render correctly
 - Test ROS data visualization in Unity 
 **Sensor testing**
 - LiDAR: 360° scan shape validation
 - Depth camera: correct depth encoding
 - IMU: stable orientation noise model 
 **Agent → Simulation loop tests**
 - Ensure robots receive controller commands from ROS
 - Validate sensor feedback loop timing 
 **APA & formatting tests**
 - Citation format verified
 - Diagrams conform to book visual theme 
------------------------------------------------------------ 
7. ORGANIZE BY PHASES 
------------------------------------------------------------ 
PHASE 1: RESEARCH
 - Study Gazebo physics engines
 - Review Unity rendering pipelines
 - Collect sensor modeling academic references 
PHASE 2: FOUNDATION
 - Build Docusaurus module folder
 - Create base Gazebo world + Unity scene
 - Prepare Neon Cyber AI–style diagrams
 - Adapt the humanoid URDF from Module 1 for Gazebo, adding necessary plugins for physics and sensors 
PHASE 3: ANALYSIS
 - Compare physics & rendering tradeoffs
 - Evaluate communication bridges
 - Validate sensor simulation details
 - Document all architectural decisions 
PHASE 4: SYNTHESIS
 - Write full chapter content
 - Produce runnable sensor simulation examples
 - Generate combined digital twin architecture diagrams
 - Final APA citation review & formatting

The tasks.md should be immediately executable - each task must be specific enough that an LLM can complete it without additional context.

## Task Generation Rules

**CRITICAL**: Tasks MUST be organized by user story to enable independent implementation and testing.

**Tests are OPTIONAL**: Only generate test tasks if explicitly requested in the feature specification or if user requests TDD approach.

### Checklist Format (REQUIRED)

Every task MUST strictly follow this format:

```text
- [ ] [TaskID] [P?] [Story?] Description with file path
```

**Format Components**:

1. **Checkbox**: ALWAYS start with `- [ ]` (markdown checkbox)
2. **Task ID**: Sequential number (T001, T002, T003...) in execution order
3. **[P] marker**: Include ONLY if task is parallelizable (different files, no dependencies on incomplete tasks)
4. **[Story] label**: REQUIRED for user story phase tasks only
   - Format: [US1], [US2], [US3], etc. (maps to user stories from spec.md)
   - Setup phase: NO story label
   - Foundational phase: NO story label  
   - User Story phases: MUST have story label
   - Polish phase: NO story label
5. **Description**: Clear action with exact file path

**Examples**:

- ✅ CORRECT: `- [ ] T001 Create project structure per implementation plan`
- ✅ CORRECT: `- [ ] T005 [P] Implement authentication middleware in src/middleware/auth.py`
- ✅ CORRECT: `- [ ] T012 [P] [US1] Create User model in src/models/user.py`
- ✅ CORRECT: `- [ ] T014 [US1] Implement UserService in src/services/user_service.py`
- ❌ WRONG: `- [ ] Create User model` (missing ID and Story label)
- ❌ WRONG: `T001 [US1] Create model` (missing checkbox)
- ❌ WRONG: `- [ ] [US1] Create User model` (missing Task ID)
- ❌ WRONG: `- [ ] T001 [US1] Create model` (missing file path)

### Task Organization

1. **From User Stories (spec.md)** - PRIMARY ORGANIZATION:
   - Each user story (P1, P2, P3...) gets its own phase
   - Map all related components to their story:
     - Models needed for that story
     - Services needed for that story
     - Endpoints/UI needed for that story
     - If tests requested: Tests specific to that story
   - Mark story dependencies (most stories should be independent)

2. **From Contracts**:
   - Map each contract/endpoint → to the user story it serves
   - If tests requested: Each contract → contract test task [P] before implementation in that story's phase

3. **From Data Model**:
   - Map each entity to the user story(ies) that need it
   - If entity serves multiple stories: Put in earliest story or Setup phase
   - Relationships → service layer tasks in appropriate story phase

4. **From Setup/Infrastructure**:
   - Shared infrastructure → Setup phase (Phase 1)
   - Foundational/blocking tasks → Foundational phase (Phase 2)
   - Story-specific setup → within that story's phase

### Phase Structure

- **Phase 1**: Setup (project initialization)
- **Phase 2**: Foundational (blocking prerequisites - MUST complete before user stories)
- **Phase 3+**: User Stories in priority order (P1, P2, P3...)
  - Within each story: Tests (if requested) → Models → Services → Endpoints → Integration
  - Each phase should be a complete, independently testable increment
- **Final Phase**: Polish & Cross-Cutting Concerns

---

As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

1) Determine Stage
   - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate Title and Determine Routing:
   - Generate Title: 3–7 words (slug for filename)
   - Route is automatically determined by stage:
     - `constitution` → `history/prompts/constitution/`
     - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)
     - `general` → `history/prompts/general/`

3) Create and Fill PHR (Shell first; fallback agent‑native)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.
   - If the script fails:
     - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)
     - Allocate an ID; compute the output path based on stage from step 2; write the file
     - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

4) Validate + report
   - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
   - On failure: warn, don't block. Skip only for `/sp.phr`.