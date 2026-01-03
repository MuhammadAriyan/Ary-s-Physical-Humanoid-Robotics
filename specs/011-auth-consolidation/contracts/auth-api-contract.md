# Auth API Contract

## /auth/sign-up [POST]

### Request
- **Content-Type**: application/json
- **Body**:
  ```json
  {
    "email": "string (required, valid email)",
    "password": "string (required, min 8 chars)",
    "name": "string (optional)"
  }
  ```

### Response
- **200 OK**:
  ```json
  {
    "user": {
      "id": "string (UUID)",
      "email": "string",
      "name": "string",
      "email_verified": "boolean"
    },
    "session": {
      "token": "string",
      "expires_at": "datetime (ISO 8601)"
    }
  }
  ```
- **400 Bad Request**: Invalid input data
- **409 Conflict**: Email already exists

### Headers
- **Set-Cookie**: auth_token={token}; HttpOnly; Secure; SameSite=Strict (optional, for browser clients)

## /auth/sign-in [POST]

### Request
- **Content-Type**: application/json
- **Body**:
  ```json
  {
    "email": "string (required, valid email)",
    "password": "string (required)"
  }
  ```

### Response
- **200 OK**:
  ```json
  {
    "user": {
      "id": "string (UUID)",
      "email": "string",
      "name": "string",
      "email_verified": "boolean"
    },
    "session": {
      "token": "string",
      "expires_at": "datetime (ISO 8601)"
    }
  }
  ```
- **400 Bad Request**: Invalid input data
- **401 Unauthorized**: Invalid credentials

### Headers
- **Set-Cookie**: auth_token={token}; HttpOnly; Secure; SameSite=Strict (optional, for browser clients)

## /auth/session [GET]

### Request
- **Authorization**: Bearer {token} OR Cookie: auth_token={token}

### Response
- **200 OK**:
  ```json
  {
    "user": {
      "id": "string (UUID)",
      "email": "string",
      "name": "string",
      "email_verified": "boolean"
    },
    "session": {
      "token": "string",
      "expires_at": "datetime (ISO 8601)"
    }
  }
  ```
- **401 Unauthorized**: Invalid or expired session

## /auth/token [POST]

### Request
- **Authorization**: Bearer {session_token} OR Cookie: auth_token={token}
- **Content-Type**: application/json
- **Body**:
  ```json
  {
    "purpose": "string (optional, e.g., 'backend_api_access')"
  }
  ```

### Response
- **200 OK**:
  ```json
  {
    "token": "string (JWT)",
    "expires_at": "datetime (ISO 8601)"
  }
  ```
- **401 Unauthorized**: Invalid or expired session

## /auth/sign-out [POST]

### Request
- **Authorization**: Bearer {token} OR Cookie: auth_token={token}

### Response
- **200 OK**:
  ```json
  {
    "success": true
  }
  ```
- **401 Unauthorized**: Invalid session

### Headers
- **Set-Cookie**: auth_token=; HttpOnly; Secure; SameSite=Strict; Max-Age=0 (to clear cookie)