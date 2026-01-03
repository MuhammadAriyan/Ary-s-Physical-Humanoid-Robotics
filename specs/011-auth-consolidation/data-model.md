# Data Model: Auth Consolidation

## User Entity
- **id**: UUID (Primary Key)
- **email**: String (Unique, Required) - User's email address for authentication
- **password_hash**: String (Required) - Bcrypt hash of user's password
- **name**: String (Optional) - User's display name
- **email_verified**: Boolean (Default: false) - Whether the email has been verified
- **created_at**: DateTime (Auto-generated) - When the user was created
- **updated_at**: DateTime (Auto-generated) - When the user was last updated
- **is_active**: Boolean (Default: true) - Whether the account is active

## Session Entity
- **id**: UUID (Primary Key)
- **user_id**: UUID (Foreign Key to User) - Reference to the user who owns the session
- **token**: String (Unique, Required) - Session token for authentication
- **expires_at**: DateTime (Required) - When the session expires
- **created_at**: DateTime (Auto-generated) - When the session was created
- **updated_at**: DateTime (Auto-generated) - When the session was last updated
- **user_agent**: String (Optional) - Browser/device information
- **ip_address**: String (Optional) - IP address of the session origin

## Account Entity (for social login)
- **id**: UUID (Primary Key)
- **user_id**: UUID (Foreign Key to User) - Reference to the user who owns the account
- **provider**: String (Required) - Social provider name (google, github, etc.)
- **provider_account_id**: String (Required) - Unique ID from the provider
- **provider_data**: JSON (Optional) - Additional data from the provider
- **created_at**: DateTime (Auto-generated) - When the account link was created
- **updated_at**: DateTime (Auto-generated) - When the account link was last updated

## Validation Rules
- User email must be valid email format
- User email must be unique across all users
- Password must meet minimum security requirements (8+ characters)
- Session tokens must be unique
- Session expiration must be in the future
- User ID in Account must reference an existing User

## State Transitions
- User: Inactive → Active (when email is verified)
- Session: Active → Expired (when current time > expires_at)
- User: Active → Inactive (administrative action)