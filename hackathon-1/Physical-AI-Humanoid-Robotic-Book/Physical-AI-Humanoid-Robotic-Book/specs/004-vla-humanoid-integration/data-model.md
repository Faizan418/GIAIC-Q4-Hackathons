# Data Models: Vision-Language-Action (VLA) System

## Entity: ObjectPercept
**Description**: Represents a detected object in the environment with spatial and semantic information

### Fields
- `id` (string): Unique identifier for the object in the current session
- `name` (string): Human-readable name of the object
- `category` (string): Semantic category (e.g., "cup", "chair", "person")
- `pose` (Pose3D): 6D pose in world coordinates (position: x, y, z; orientation: qx, qy, qz, qw)
- `confidence` (float): Detection confidence (0.0-1.0)
- `bounding_box` (BoundingBox2D): 2D bounding box in camera image (x, y, width, height)
- `features` (FeatureVector): High-dimensional feature representation
- `timestamp` (datetime): Time of detection
- `tracked` (boolean): Whether object is being actively tracked

### Relationships
- Connected to `EnvironmentMap` via `contained_objects`
- Connected to `ActionPlan` via `target_object`

### Validation Rules
- `confidence` must be between 0.0 and 1.0
- `pose` quaternion must be normalized
- `id` must be unique within current session

## Entity: LanguageCommand
**Description**: Represents a natural language command or query from a user

### Fields
- `id` (string): Unique identifier for the command
- `text` (string): Original text of the command
- `intent` (IntentType): Classified intent (e.g., "FETCH_OBJECT", "NAVIGATE", "ANSWER_QUESTION")
- `entities` (Array[Entity]): Extracted semantic entities with types and values
- `context` (Context): Conversation context and history
- `priority` (Priority): Execution priority (LOW, MEDIUM, HIGH, CRITICAL)
- `timestamp` (datetime): Time of command receipt
- `resolved` (boolean): Whether command has been processed

### Relationships
- Connected to `ActionPlan` via `associated_plan`
- Connected to `Conversation` via `conversation_id`

### Validation Rules
- `text` must not be empty
- `intent` must be a valid IntentType
- `priority` must be one of defined priority levels

## Entity: ActionPlan
**Description**: Represents a planned sequence of actions to achieve a goal

### Fields
- `id` (string): Unique identifier for the plan
- `task` (TaskType): High-level task type
- `steps` (Array[ActionStep]): Ordered sequence of action steps
- `constraints` (Array[Constraint]): Motion and environmental constraints
- `success_criteria` (Array[Criterion]): Conditions for plan success
- `estimated_time` (Duration): Estimated execution time
- `confidence` (float): Confidence in successful execution (0.0-1.0)
- `status` (PlanStatus): Current status (PLANNING, EXECUTING, COMPLETED, FAILED)
- `timestamp` (datetime): Time of plan creation

### Relationships
- Connected to `LanguageCommand` via `source_command`
- Connected to `RobotState` via `required_state`

### Validation Rules
- `steps` array must not be empty
- `confidence` must be between 0.0 and 1.0
- `status` must be a valid PlanStatus

## Entity: ActionStep
**Description**: A single step within an action plan

### Fields
- `id` (string): Unique identifier for the step
- `type` (StepType): Type of action (MOVE, GRASP, SPEAK, WAIT, etc.)
- `parameters` (Parameters): Action-specific parameters
- `preconditions` (Array[Condition]): Conditions that must be true before execution
- `effects` (Array[Effect]): Expected effects of the action
- `timeout` (Duration): Maximum time to complete the step
- `recovery_action` (ActionStep): Action to take if this step fails

### Relationships
- Connected to `ActionPlan` via `plan_id`

### Validation Rules
- `type` must be a valid StepType
- `timeout` must be positive

## Entity: EnvironmentMap
**Description**: Represents the robot's understanding of its environment

### Fields
- `id` (string): Unique identifier for the map
- `type` (MapType): Type of map (TOPOLOGICAL, METRIC, SEMANTIC)
- `data` (MapData): Actual map data (grid, graph, or semantic representation)
- `objects` (Array[ObjectPercept]): Known objects in the environment
- `obstacles` (Array[Obstacle]): Static and dynamic obstacles
- `navigable_areas` (Array[Area]): Navigable regions
- `timestamp` (datetime): Time of last update
- `resolution` (float): Map resolution (for metric maps)

### Relationships
- Connected to `RobotState` via `current_environment`
- Connected to `ObjectPercept` via `contained_objects`

### Validation Rules
- `resolution` must be positive
- `data` format must match `type`

## Entity: RobotState
**Description**: Represents the current state of the robot

### Fields
- `id` (string): Unique identifier for the state snapshot
- `joint_states` (Array[JointState]): Current joint positions, velocities, efforts
- `end_effector_pose` (Pose3D): Current pose of end effector
- `base_pose` (Pose3D): Current pose of robot base
- `battery_level` (float): Current battery level (0.0-1.0)
- `operational_mode` (Mode): Current operational mode
- `safety_status` (SafetyStatus): Current safety status
- `timestamp` (datetime): Time of state capture

### Relationships
- Connected to `ActionPlan` via `required_state`
- Connected to `EnvironmentMap` via `current_environment`

### Validation Rules
- `battery_level` must be between 0.0 and 1.0
- `joint_states` must match robot's DOF

## Entity: Conversation
**Description**: Represents a conversation session with a user

### Fields
- `id` (string): Unique identifier for the conversation
- `participants` (Array[Participant]): List of participants
- `context` (Context): Shared context for the conversation
- `history` (Array[ConversationTurn]): Chronological list of conversation turns
- `active` (boolean): Whether conversation is currently active
- `start_time` (datetime): Time conversation started
- `last_activity` (datetime): Time of last activity

### Relationships
- Connected to `LanguageCommand` via `conversation_id`
- Connected to `RobotState` via `robot_participant`

### Validation Rules
- `history` must maintain chronological order
- `active` must be consistent with `last_activity`

## Entity: ConversationTurn
**Description**: A single turn in a conversation

### Fields
- `id` (string): Unique identifier for the turn
- `speaker` (SpeakerType): Who spoke (HUMAN, ROBOT)
- `text` (string): Text of the utterance
- `intent` (IntentType): Intent of the utterance
- `timestamp` (datetime): Time of the utterance
- `confidence` (float): Confidence in interpretation (0.0-1.0)

### Relationships
- Connected to `Conversation` via `conversation_id`

### Validation Rules
- `confidence` must be between 0.0 and 1.0
- `speaker` must be valid SpeakerType

## State Transitions

### ActionPlan State Transitions
- PLANNING → EXECUTING: When plan execution begins
- EXECUTING → COMPLETED: When all steps complete successfully
- EXECUTING → FAILED: When a step fails and no recovery possible
- EXECUTING → PAUSED: When execution is temporarily suspended
- PAUSED → EXECUTING: When execution resumes

### RobotState Safety Transitions
- NORMAL → WARNING: When safety threshold approached
- WARNING → EMERGENCY: When safety threshold exceeded
- EMERGENCY → NORMAL: When safety condition resolved

## Indexes and Performance Considerations

### Spatial Indexes
- R-tree for efficient spatial queries on ObjectPercept
- KD-tree for nearest neighbor searches in EnvironmentMap

### Temporal Indexes
- Time-series database for RobotState history
- B-tree for chronological ordering of ConversationTurn

### Semantic Indexes
- Inverted index for text-based queries on LanguageCommand
- Vector index for similarity search on ObjectPercept features
