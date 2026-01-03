# Quickstart Guide: Auth Service Integration

## Setup

### 1. Install Dependencies
```bash
pip install python-jose[cryptography] passlib[bcrypt] python-multipart
```

### 2. Environment Variables
```bash
# JWT Configuration
JWT_SECRET_KEY=your-super-secret-jwt-key
JWT_ALGORITHM=HS256
JWT_ACCESS_TOKEN_EXPIRE_MINUTES=30

# Password Hashing
PASSWORD_HASH_SCHEMES=["bcrypt"]
PASSWORD_HASH_DEPRECATED="auto"

# Database
DATABASE_URL=postgresql://user:password@localhost/dbname
```

## Running the Auth Service

### 1. Start the FastAPI Server
```bash
uvicorn main:app --host 0.0.0.0 --port 8000
```

### 2. API Endpoints Available
- `POST /auth/sign-up` - User registration
- `POST /auth/sign-in` - User login
- `GET /auth/session` - Get current session
- `POST /auth/token` - Generate JWT for backend access
- `POST /auth/sign-out` - Logout

## Frontend Integration

### 1. User Registration
```javascript
const response = await fetch('/auth/sign-up', {
  method: 'POST',
  headers: {
    'Content-Type': 'application/json',
  },
  body: JSON.stringify({
    email: 'user@example.com',
    password: 'securepassword',
    name: 'User Name'
  })
});
```

### 2. User Login
```javascript
const response = await fetch('/auth/sign-in', {
  method: 'POST',
  headers: {
    'Content-Type': 'application/json',
  },
  body: JSON.stringify({
    email: 'user@example.com',
    password: 'securepassword'
  })
});
```

### 3. Session Verification
```javascript
const response = await fetch('/auth/session', {
  method: 'GET',
  credentials: 'include'  // Include cookies
});
```

## Database Schema

The auth service uses the following tables:
- `users` - Stores user information
- `sessions` - Manages active user sessions
- `accounts` - Links social provider accounts to users

## Testing

### Run Tests
```bash
pytest tests/auth/
```

### Test Endpoints
- Use the test endpoints to verify auth functionality
- Check that JWT tokens are properly generated and validated
- Verify CORS configuration works with your frontend domain