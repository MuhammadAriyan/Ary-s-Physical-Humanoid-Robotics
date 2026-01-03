# Data Model: Auth Service Migration

## Entities

### Auth Service Configuration
- **Name**: Auth Service Configuration
- **Description**: Configuration parameters for the Better Auth service
- **Fields**:
  - baseURL: string (the Replit URL for the auth service)
  - secret: string (Better Auth secret key)
  - databaseURL: string (connection string for database)
  - corsOrigins: array of strings (allowed origins for CORS)
  - sessionConfig: object (session management settings)

### Environment Variables
- **Name**: Environment Variables
- **Description**: Secure configuration values for the auth service
- **Fields**:
  - BETTER_AUTH_SECRET: string (auth secret)
  - BETTER_AUTH_URL: string (base URL for auth service)
  - DATABASE_URL: string (database connection string)
  - GITHUB_ID: string (OAuth provider ID)
  - GITHUB_SECRET: string (OAuth provider secret)

### Frontend Applications
- **Name**: Frontend Applications
- **Description**: Applications that consume the auth service
- **Fields**:
  - name: string (application name)
  - url: string (application URL)
  - corsEnabled: boolean (whether CORS is configured for this app)