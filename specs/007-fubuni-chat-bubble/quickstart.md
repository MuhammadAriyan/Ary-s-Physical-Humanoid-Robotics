# Quickstart: Fubuni Chat Bubble

## Prerequisites
- Python 3.11+
- Node.js 18+ (for Docusaurus development)
- Access to OpenRouter API (or compatible OpenAI-compatible provider)
- Docusaurus 3.x installed
- Neon Serverless Postgres database instance

## Backend Setup

1. **Install Python dependencies**:
   ```bash
   cd backend
   pip install -r requirements.txt
   ```

2. **Set up environment variables**:
   ```bash
   # Create .env file in backend directory
   OPENROUTER_API_KEY=your_openrouter_api_key
   NEON_DATABASE_URL=your_neon_database_connection_string
   ```

3. **Initialize the database**:
   ```bash
   # Run database migrations to create required tables
   cd backend
   python -m app.database.init
   ```

4. **Run the backend**:
   ```bash
   cd backend
   uvicorn app.main:app --reload --port 8000
   ```

## Frontend Setup

1. **Install frontend dependencies**:
   ```bash
   npm install
   ```

2. **Configure Docusaurus**:
   Update your `docusaurus.config.js` to include the Fubuni chat component.

3. **Start Docusaurus development server**:
   ```bash
   npm start
   ```

## Environment Variables

### Backend (.env)
```
OPENROUTER_API_KEY=your_openrouter_api_key
NEON_DATABASE_URL=your_neon_database_connection_string
OPENAI_BASE_URL=https://openrouter.ai/api/v1/chat/completions
APP_ENV=development  # or production
BACKEND_CORS_ORIGINS=["http://localhost:3000", "http://localhost:3001"]
```

### Frontend (.env)
```
REACT_APP_BACKEND_URL=http://localhost:8000  # or your backend URL
```

## API Usage Example

### Sending a message to Fubuni:
```javascript
// Using fetch
const response = await fetch('http://localhost:8000/api/chat', {
  method: 'POST',
  headers: {
    'Content-Type': 'application/json',
  },
  body: JSON.stringify({
    message: 'Hello Fubuni!',
    session_id: 'unique-session-id', // optional
    stream: true
  })
});

// Handle streaming response
const reader = response.body.getReader();
const decoder = new TextDecoder();

while (true) {
  const { done, value } = await reader.read();
  if (done) break;

  const chunk = decoder.decode(value);
  // Process SSE events
  console.log(chunk);
}
```

## Docusaurus Integration

The chat component is integrated via Docusaurus swizzling. The component will automatically:
- Inject the floating bubble in the bottom-right corner
- Handle theme consistency (light/dark mode)
- Connect to the backend API
- Manage session state

## Troubleshooting

1. **Chat bubble not appearing**: Check that the swizzled component is properly injected in your Docusaurus theme
2. **API connection errors**: Verify backend is running and API key is correct
3. **Streaming not working**: Ensure SSE endpoint is properly configured
4. **Theme mismatch**: Check that Infima classes are properly applied