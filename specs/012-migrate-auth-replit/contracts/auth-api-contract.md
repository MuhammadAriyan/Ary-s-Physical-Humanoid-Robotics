# Auth API Contract

## Overview
API contract for Better Auth service running on Replit. This document describes the endpoints that will remain consistent during the migration from Koyeb to Replit.

## Base URL
`https://[repl-name].[username].repl.co`

## Authentication Endpoints

### POST /api/auth/register
Register a new user account.

**Request**:
```json
{
  "email": "user@example.com",
  "password": "securePassword123"
}
```

**Response**:
- 200: User registered successfully
- 400: Invalid input
- 409: User already exists

### POST /api/auth/login
Authenticate a user and create a session.

**Request**:
```json
{
  "email": "user@example.com",
  "password": "securePassword123"
}
```

**Response**:
- 200: Login successful, session created
- 400: Invalid input
- 401: Invalid credentials

### GET /api/auth/me
Get current user information.

**Headers**:
- Authorization: Bearer [token] (if using token auth)

**Response**:
- 200: User information
- 401: Unauthorized

### POST /api/auth/logout
End the current user session.

**Response**:
- 200: Logout successful
- 401: Unauthorized

## CORS Configuration
The service will be configured to allow requests from specified origins as configured in the Better Auth setup.

## Error Format
All error responses follow the format:
```json
{
  "error": {
    "message": "Descriptive error message",
    "code": "ERROR_CODE"
  }
}
```