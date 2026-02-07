# Data Model: Frontend Chat Integration

## Entity: ChatMessage

**Description**: Represents a single message in the chat conversation

**Fields**:
- `id`: string (unique identifier for the message)
- `sender`: 'user' | 'system' (indicates message origin)
- `content`: string (the actual message text)
- `timestamp`: Date (when the message was created/sent)
- `status`: 'pending' | 'sent' | 'delivered' | 'error' (message transmission status)
- `parentId?`: string (optional, for threading in multi-turn conversations)

**Validation Rules**:
- `content` must not be empty or exceed 2000 characters
- `sender` must be one of the allowed values
- `timestamp` must be a valid date

## Entity: ConversationSession

**Description**: Represents the current chat session with message history

**Fields**:
- `sessionId`: string (unique identifier for the session)
- `messages`: ChatMessage[] (ordered array of messages in the conversation)
- `createdAt`: Date (when the session was created)
- `lastActiveAt`: Date (when the last message was sent/received)
- `isActive`: boolean (whether the session is currently active)

**Validation Rules**:
- `messages` array must not exceed 100 messages
- `sessionId` must be unique per active session

## Entity: TextSelection

**Description**: Represents highlighted text from the textbook that can trigger contextual queries

**Fields**:
- `id`: string (unique identifier)
- `content`: string (the selected/highlighted text)
- `context`: string (surrounding context of the selection)
- `position`: { start: number, end: number } (character positions in the source)
- `timestamp`: Date (when the selection was made)
- `sourceUrl`: string (URL of the page where text was selected)

**Validation Rules**:
- `content` must not be empty and should be between 10-1000 characters
- `position` values must be valid indices within the context
- `sourceUrl` must be a valid URL

## State Transitions

### ChatMessage State Transitions
```
pending → sent → delivered
   ↓
  error (with retry capability)
```

### ConversationSession State Transitions
```
active ↔ inactive (based on user interaction and timeout)
```

## Relationships

- ConversationSession contains multiple ChatMessage entities
- TextSelection can initiate a new ChatMessage with 'user' sender type
- ChatMessage can reference previous messages via parentId for conversation threading
