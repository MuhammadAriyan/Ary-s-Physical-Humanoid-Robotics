"""
Authentication middleware for validating Better Auth JWT tokens.
Validates tokens by calling the auth-service session endpoint.
"""

from typing import Optional
from dataclasses import dataclass
from fastapi import Depends, HTTPException, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
import httpx
from ..config.settings import settings


# HTTP Bearer token scheme
security = HTTPBearer(auto_error=False)


@dataclass
class AuthUser:
    """Authenticated user data from Better Auth session."""
    id: str
    email: str
    name: Optional[str] = None
    image: Optional[str] = None
    email_verified: bool = False


async def verify_token(token: str) -> Optional[AuthUser]:
    """
    Verify the session token by calling the auth-service.

    Better Auth uses session tokens, not JWTs, so we need to
    validate with the auth service's session endpoint.
    """
    try:
        async with httpx.AsyncClient() as client:
            response = await client.get(
                f"{settings.auth_service_url}/api/auth/get-session",
                headers={
                    "Authorization": f"Bearer {token}",
                },
                timeout=10.0,
            )

            if response.status_code != 200:
                return None

            data = response.json()

            # Check if session is valid
            if not data or not data.get("user"):
                return None

            user_data = data["user"]
            return AuthUser(
                id=user_data["id"],
                email=user_data["email"],
                name=user_data.get("name"),
                image=user_data.get("image"),
                email_verified=user_data.get("emailVerified", False),
            )
    except httpx.RequestError as e:
        print(f"Auth service request error: {e}")
        return None
    except Exception as e:
        print(f"Token verification error: {e}")
        return None


async def get_current_user(
    credentials: Optional[HTTPAuthorizationCredentials] = Depends(security),
) -> Optional[AuthUser]:
    """
    Optional dependency to get the current authenticated user.
    Returns None if not authenticated (for mixed public/protected endpoints).
    """
    if not credentials:
        return None

    user = await verify_token(credentials.credentials)
    return user


async def require_auth(
    credentials: Optional[HTTPAuthorizationCredentials] = Depends(security),
) -> AuthUser:
    """
    Required dependency that ensures user is authenticated.
    Raises 401 Unauthorized if no valid token is provided.
    """
    if not credentials:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authentication required",
            headers={"WWW-Authenticate": "Bearer"},
        )

    user = await verify_token(credentials.credentials)

    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired session",
            headers={"WWW-Authenticate": "Bearer"},
        )

    return user
