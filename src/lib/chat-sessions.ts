/**
 * Chat session management API helpers
 * Uses Better Auth session tokens for authentication
 */

import { getSessionToken } from './auth-client';

// Get the API base URL based on environment
function getApiBaseUrl(): string {
  if (typeof window === 'undefined') {
    return 'https://gorgeous-deeanne-ary-s-88e09c71.koyeb.app';
  }
  return window.location.hostname === 'localhost'
    ? 'http://localhost:8000'
    : 'https://gorgeous-deeanne-ary-s-88e09c71.koyeb.app';
}

// Types for chat sessions
export interface ChatSession {
  id: string;
  title: string | null;
  created_at: string;
  last_interaction: string;
  is_active: boolean;
  message_count: number;
}

export interface ChatMessage {
  id: string;
  sender: 'user' | 'fubuni';
  content: string;
  chat_session_id: string;
  sequence_number: number;
  timestamp: string;
}

export interface SessionListResponse {
  sessions: ChatSession[];
  total: number;
}

export interface SessionWithMessages {
  session: ChatSession;
  messages: ChatMessage[];
}

// API Error class
export class ChatSessionError extends Error {
  constructor(
    message: string,
    public status: number,
    public code?: string
  ) {
    super(message);
    this.name = 'ChatSessionError';
  }
}

// Helper to make authenticated requests
async function authenticatedFetch(
  endpoint: string,
  options: RequestInit = {}
): Promise<Response> {
  const token = await getSessionToken();

  if (!token) {
    throw new ChatSessionError('Not authenticated', 401, 'UNAUTHORIZED');
  }

  const apiUrl = getApiBaseUrl();
  const response = await fetch(`${apiUrl}${endpoint}`, {
    ...options,
    headers: {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${token}`,
      ...options.headers,
    },
  });

  if (!response.ok) {
    const errorData = await response.json().catch(() => ({}));
    throw new ChatSessionError(
      errorData.detail || `Request failed with status ${response.status}`,
      response.status,
      errorData.code
    );
  }

  return response;
}

/**
 * List all chat sessions for the current user
 * Sessions are ordered by last_interaction DESC (most recent first)
 */
export async function listSessions(): Promise<SessionListResponse> {
  const response = await authenticatedFetch('/api/chat/sessions');
  return response.json();
}

/**
 * Create a new empty chat session
 */
export async function createSession(): Promise<ChatSession> {
  const response = await authenticatedFetch('/api/chat/sessions', {
    method: 'POST',
  });
  return response.json();
}

/**
 * Get a chat session with all its messages
 */
export async function getSessionMessages(sessionId: string): Promise<SessionWithMessages> {
  const response = await authenticatedFetch(`/api/chat/history/${sessionId}`);
  return response.json();
}

/**
 * Delete a chat session and all its messages
 */
export async function deleteSession(sessionId: string): Promise<{ success: boolean }> {
  const response = await authenticatedFetch(`/api/chat/sessions/${sessionId}`, {
    method: 'DELETE',
  });
  return response.json();
}

/**
 * Send a message to a chat session (or create a new one)
 */
export async function sendMessage(
  message: string,
  sessionId?: string
): Promise<{
  response: string;
  session_id: string;
  timestamp: string;
  chapter?: string;
  section?: string;
  should_navigate?: boolean;
}> {
  const response = await authenticatedFetch('/api/chat', {
    method: 'POST',
    body: JSON.stringify({
      message,
      session_id: sessionId,
      stream: false,
    }),
  });
  return response.json();
}

/**
 * Format relative time (e.g., "2 hours ago")
 */
export function formatRelativeTime(dateString: string): string {
  const date = new Date(dateString);
  const now = new Date();
  const diffMs = now.getTime() - date.getTime();
  const diffSeconds = Math.floor(diffMs / 1000);
  const diffMinutes = Math.floor(diffSeconds / 60);
  const diffHours = Math.floor(diffMinutes / 60);
  const diffDays = Math.floor(diffHours / 24);

  if (diffSeconds < 60) {
    return 'just now';
  } else if (diffMinutes < 60) {
    return `${diffMinutes} minute${diffMinutes === 1 ? '' : 's'} ago`;
  } else if (diffHours < 24) {
    return `${diffHours} hour${diffHours === 1 ? '' : 's'} ago`;
  } else if (diffDays < 7) {
    return `${diffDays} day${diffDays === 1 ? '' : 's'} ago`;
  } else {
    return date.toLocaleDateString();
  }
}
