---

description: "Task list for Todo In-Memory Python Console App implementation"
---

# Tasks: Todo In-Memory Python Console App

**Input**: Design documents from `/specs/001-todo-app/`
**Prerequisites**: plan.md (required), spec.md (required), research.md

**Tests**: Tests are NOT included in this task breakdown as they were not explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: `src/`, `tests/` at repository root
- Paths shown below use single project structure per plan.md

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create project directory structure (src/, tests/unit/, tests/integration/, tests/edge_cases/)
- [x] T002 Initialize UV project with pyproject.toml for Python 3.13+
- [x] T003 [P] Add optional dependencies to pyproject.toml (colorama, pytest)
- [x] T004 [P] Create empty module files: src/models.py, src/storage.py, src/services.py, src/cli.py, src/main.py

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T005 [P] Implement Task dataclass in src/models.py with basic fields (id, title, completed, created_at)
- [x] T006 [P] Implement in-memory storage initialization in src/storage.py (tasks_list, tasks_index, next_id)
- [x] T007 Implement get_next_id() function in src/storage.py for unique ID generation
- [x] T008 Implement add_task_to_storage() function in src/storage.py
- [x] T009 [P] Implement get_task_by_id() function in src/storage.py
- [x] T010 [P] Implement get_all_tasks() function in src/storage.py
- [x] T011 [P] Implement delete_task_from_storage() function in src/storage.py
- [x] T012 [P] Implement validate_title() function in src/services.py
- [x] T013 Create basic CLI menu structure in src/cli.py (display_main_menu, get_menu_choice)
- [x] T014 Implement application entry point in src/main.py

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Basic Task Management (Priority: P1) 🎯 MVP

**Goal**: Implement foundational CRUD operations for task management

**Independent Test**: Create a task, view it in the list, update its details, mark it complete, then delete it. System provides basic task tracking value.

### Implementation for User Story 1

- [x] T015 [P] [US1] Implement create_task() function in src/services.py (uses storage layer)
- [x] T016 [P] [US1] Implement read_task() function in src/services.py
- [x] T017 [P] [US1] Implement read_all_tasks() function in src/services.py
- [x] T018 [US1] Implement update_task() function in src/services.py
- [x] T019 [US1] Implement delete_task() function in src/services.py (with storage cleanup)
- [x] T020 [US1] Implement toggle_completion() function in src/services.py
- [x] T021 [US1] Implement handle_add_task() in src/cli.py (prompts for title, calls create_task)
- [x] T022 [US1] Implement handle_view_tasks() in src/cli.py (displays task list with formatting)
- [x] T023 [US1] Implement display_task_list() helper in src/cli.py (table formatting)
- [x] T024 [US1] Implement handle_update_task() in src/cli.py (prompts for ID and new title)
- [x] T025 [US1] Implement handle_delete_task() in src/cli.py (with confirmation prompt)
- [x] T026 [US1] Implement handle_toggle_completion() in src/cli.py (prompts for ID, toggles status)
- [x] T027 [US1] Implement get_user_input() helper in src/cli.py (input validation wrapper)
- [x] T028 [US1] Implement confirm_action() helper in src/cli.py (yes/no confirmation)
- [x] T029 [US1] Wire up menu options 1-6 in src/cli.py run_app() loop (Add, View, Update, Delete, Toggle, Exit)
- [x] T030 [US1] Add input validation and error messages for empty titles in src/cli.py handlers
- [x] T031 [US1] Add error handling for invalid task IDs in src/cli.py handlers

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Task Organization (Priority: P2)

**Goal**: Add priorities, tags, search, filter, and sort capabilities for task organization

**Independent Test**: Create multiple tasks with different priorities and tags, verify search returns correct results, filters show appropriate subsets, and sorting reorders tasks correctly.

### Implementation for User Story 2

- [x] T032 [P] [US2] Extend Task dataclass in src/models.py to add priority field (default: "Medium")
- [x] T033 [P] [US2] Extend Task dataclass in src/models.py to add tags field (default: empty list)
- [x] T034 [US2] Update create_task() in src/services.py to accept priority and tags parameters
- [x] T035 [US2] Update update_task() in src/services.py to support priority and tags updates
- [x] T036 [P] [US2] Implement validate_priority() function in src/services.py
- [x] T037 [P] [US2] Implement search_tasks() function in src/services.py (keyword search, case-insensitive)
- [x] T038 [P] [US2] Implement filter_by_status() function in src/services.py (completed/pending)
- [x] T039 [P] [US2] Implement filter_by_priority() function in src/services.py
- [x] T040 [P] [US2] Implement filter_by_tag() function in src/services.py
- [x] T041 [P] [US2] Implement sort_tasks() function in src/services.py (by priority, date, title)
- [x] T042 [US2] Update handle_add_task() in src/cli.py to prompt for priority and tags
- [x] T043 [US2] Update handle_update_task() in src/cli.py to support priority and tags updates
- [x] T044 [US2] Update display_task_list() in src/cli.py to show priority and tags
- [x] T045 [US2] Implement handle_search() in src/cli.py (prompts for keyword, displays results)
- [x] T046 [US2] Implement handle_filter() in src/cli.py (menu for status, priority, tag filters)
- [x] T047 [US2] Implement handle_sort() in src/cli.py (menu for sort options: priority, date, title)
- [x] T048 [US2] Wire up menu options 7-9 in src/cli.py run_app() loop (Search, Filter, Sort)
- [x] T049 [US2] Add priority validation and error messages in src/cli.py handlers

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Advanced Scheduling (Priority: P3)

**Goal**: Implement due dates, reminders, and recurring tasks for time-sensitive task management

**Independent Test**: Create a task with a due date and verify reminders appear, then create a recurring task and verify it auto-regenerates on schedule.

### Implementation for User Story 3

- [x] T050 [P] [US3] Extend Task dataclass in src/models.py to add due_date field (Optional[datetime])
- [x] T051 [P] [US3] Create RecurrencePattern dataclass in src/models.py (frequency, original_due_date)
- [x] T052 [P] [US3] Extend Task dataclass in src/models.py to add recurrence field (Optional[RecurrencePattern])
- [x] T053 [US3] Update create_task() in src/services.py to accept due_date parameter
- [x] T054 [P] [US3] Implement validate_date() function in src/services.py (parse datetime string)
- [x] T055 [P] [US3] Implement set_due_date() function in src/services.py
- [x] T056 [P] [US3] Implement set_recurrence() function in src/services.py
- [x] T057 [P] [US3] Implement check_reminders() function in src/services.py (returns tasks due now)
- [x] T058 [P] [US3] Implement is_overdue() helper in src/services.py (checks if task past due)
- [x] T059 [US3] Implement calculate_next_occurrence() helper in src/services.py (Daily/Weekly/Monthly)
- [x] T060 [US3] Implement regenerate_recurring_task() function in src/services.py (called on toggle_completion)
- [x] T061 [US3] Update toggle_completion() in src/services.py to handle recurring task regeneration
- [x] T062 [US3] Implement display_reminders() in src/cli.py (shows tasks due now)
- [x] T063 [US3] Update display_main_menu() in src/cli.py to call display_reminders() before menu
- [x] T064 [US3] Update display_task_list() in src/cli.py to show due dates and overdue markers
- [x] T065 [US3] Implement handle_set_due_date() in src/cli.py (prompts for task ID and date)
- [x] T066 [US3] Implement handle_set_recurrence() in src/cli.py (prompts for ID, frequency, start date)
- [x] T067 [US3] Wire up menu options 10-11 in src/cli.py run_app() loop (Set Due Date, Set Recurrence)
- [x] T068 [US3] Add date format validation and error messages in src/cli.py handlers
- [x] T069 [US3] Add recurring task regeneration logic test (verify new task created on completion)

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T070 [P] Add colorama color formatting for priority indicators in src/cli.py (High=red, Medium=yellow, Low=green)
- [x] T071 [P] Add completion status indicators (checkmarks) in src/cli.py display functions
- [x] T072 [P] Improve error message formatting and consistency across all CLI handlers
- [x] T073 [P] Add input validation helpers for common patterns (non-empty string, valid ID range)
- [x] T074 [P] Add docstrings to all public functions in src/services.py
- [x] T075 [P] Add docstrings to all public functions in src/cli.py
- [x] T076 [P] Add type hints to all function signatures across all modules
- [x] T077 Create README.md with setup instructions, UV commands, and feature list
- [x] T078 Create CLAUDE.md with Claude Code workflow and agent usage guide
- [x] T079 [P] Add edge case handling for 1000+ tasks performance testing
- [x] T080 Final integration test: Run through all features end-to-end and verify success criteria

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Phase 6)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Extends US1 but is independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Extends US1 but is independently testable

### Within Each User Story

- Models before services (data structures must exist)
- Services before CLI handlers (business logic must exist)
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel (T002-T004)
- All Foundational tasks marked [P] can run in parallel within Phase 2 (T005-T012)
- Once Foundational phase completes, all three user stories can start in parallel (if team capacity allows)
- Models within a story marked [P] can run in parallel (e.g., T032-T033 in US2)
- Service functions marked [P] within a story can run in parallel (e.g., T037-T041 in US2)
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# After Foundational phase completes, launch these US1 tasks together:
Task T015: "Implement create_task() in src/services.py"
Task T016: "Implement read_task() in src/services.py"
Task T017: "Implement read_all_tasks() in src/services.py"

# After services are complete, launch these CLI tasks together:
Task T021: "Implement handle_add_task() in src/cli.py"
Task T022: "Implement handle_view_tasks() in src/cli.py"
Task T023: "Implement display_task_list() in src/cli.py"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T004)
2. Complete Phase 2: Foundational (T005-T014) **CRITICAL - blocks all stories**
3. Complete Phase 3: User Story 1 (T015-T031)
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 (T015-T031) → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 (T032-T049) → Test independently → Deploy/Demo
4. Add User Story 3 (T050-T069) → Test independently → Deploy/Demo
5. Add Polish (T070-T080) → Final testing → Deploy/Demo
6. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together (T001-T014)
2. Once Foundational is done:
   - Developer A: User Story 1 (T015-T031)
   - Developer B: User Story 2 (T032-T049)
   - Developer C: User Story 3 (T050-T069)
3. Stories complete and integrate independently

---

## Task Summary

**Total Tasks**: 80
- Phase 1 (Setup): 4 tasks
- Phase 2 (Foundational): 10 tasks **[BLOCKING - must complete first]**
- Phase 3 (User Story 1 - Basic): 17 tasks → **MVP**
- Phase 4 (User Story 2 - Organization): 18 tasks
- Phase 5 (User Story 3 - Advanced): 20 tasks
- Phase 6 (Polish): 11 tasks

**Tasks per User Story**:
- US1 (Basic Task Management): 17 tasks
- US2 (Task Organization): 18 tasks
- US3 (Advanced Scheduling): 20 tasks

**Parallel Opportunities Identified**:
- Setup: 2 tasks (T003-T004)
- Foundational: 7 tasks (T005-T006, T009-T010, T011-T012)
- US1: 2 groups (T015-T017, T021-T023)
- US2: 6 tasks (T032-T033, T036-T041)
- US3: 8 tasks (T050-T052, T054-T058)
- Polish: 9 tasks (T070-T076, T079)

**Independent Test Criteria**:
- **US1**: Create → View → Update → Complete → Delete cycle works end-to-end
- **US2**: Create tasks with priorities/tags → Search → Filter → Sort all work correctly
- **US3**: Set due date → See reminders → Create recurring → Verify regeneration

**Suggested MVP Scope**: Phase 1-3 only (Tasks T001-T031)
- Delivers: Full CRUD operations, basic task management
- 31 tasks total
- Estimated: 2-3 implementation sessions

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Stop at any checkpoint to validate story independently
- Commit after each task or logical group
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence

---

## Format Validation

✅ All tasks follow checklist format: `- [ ] [ID] [P?] [Story?] Description with file path`
✅ All user story tasks include [US1], [US2], or [US3] labels
✅ All tasks include specific file paths
✅ Setup and Foundational phases have NO story labels
✅ Polish phase has NO story labels (cross-cutting concerns)
✅ Tasks organized by user story priority (P1 → P2 → P3)
✅ Independent test criteria defined for each user story
✅ Parallel opportunities clearly marked with [P]
✅ Dependencies documented showing execution order
