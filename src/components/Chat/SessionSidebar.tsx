import React, { useEffect, useState, useCallback } from 'react';
import {
  ChatSession,
  listSessions,
  createSession,
  deleteSession,
  formatRelativeTime,
  ChatSessionError,
} from '../../lib/chat-sessions';
import styles from './SessionSidebar.module.css';

interface SessionSidebarProps {
  activeSessionId: string | null;
  onSessionSelect: (sessionId: string) => void;
  onNewSession: (sessionId: string) => void;
  isAuthenticated: boolean;
}

export const SessionSidebar: React.FC<SessionSidebarProps> = ({
  activeSessionId,
  onSessionSelect,
  onNewSession,
  isAuthenticated,
}) => {
  const [sessions, setSessions] = useState<ChatSession[]>([]);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [isCreating, setIsCreating] = useState(false);
  const [deletingId, setDeletingId] = useState<string | null>(null);
  const [showDeleteConfirm, setShowDeleteConfirm] = useState<string | null>(null);

  // Fetch sessions when authenticated
  const fetchSessions = useCallback(async () => {
    if (!isAuthenticated) {
      setSessions([]);
      setIsLoading(false);
      return;
    }

    try {
      setIsLoading(true);
      setError(null);
      const response = await listSessions();
      setSessions(response.sessions);
    } catch (err) {
      if (err instanceof ChatSessionError) {
        setError(err.message);
      } else {
        setError('Failed to load sessions');
      }
    } finally {
      setIsLoading(false);
    }
  }, [isAuthenticated]);

  useEffect(() => {
    fetchSessions();
  }, [fetchSessions]);

  // Handle creating a new session
  const handleNewSession = useCallback(async () => {
    if (isCreating) return;

    try {
      setIsCreating(true);
      setError(null);
      const newSession = await createSession();
      setSessions((prev) => [newSession, ...prev]);
      onNewSession(newSession.id);
    } catch (err) {
      if (err instanceof ChatSessionError) {
        setError(err.message);
      } else {
        setError('Failed to create session');
      }
    } finally {
      setIsCreating(false);
    }
  }, [isCreating, onNewSession]);

  // Handle deleting a session
  const handleDelete = useCallback(async (sessionId: string) => {
    if (deletingId) return;

    try {
      setDeletingId(sessionId);
      setShowDeleteConfirm(null);
      await deleteSession(sessionId);
      setSessions((prev) => prev.filter((s) => s.id !== sessionId));

      // If we deleted the active session, select another or clear
      if (activeSessionId === sessionId) {
        const remaining = sessions.filter((s) => s.id !== sessionId);
        if (remaining.length > 0) {
          onSessionSelect(remaining[0].id);
        } else {
          onNewSession('');
        }
      }
    } catch (err) {
      if (err instanceof ChatSessionError) {
        setError(err.message);
      } else {
        setError('Failed to delete session');
      }
    } finally {
      setDeletingId(null);
    }
  }, [deletingId, activeSessionId, sessions, onSessionSelect, onNewSession]);

  // Refresh sessions (can be called externally)
  const refresh = useCallback(() => {
    fetchSessions();
  }, [fetchSessions]);

  // Expose refresh method
  useEffect(() => {
    (window as unknown as Record<string, unknown>).__refreshSessions = refresh;
    return () => {
      delete (window as unknown as Record<string, unknown>).__refreshSessions;
    };
  }, [refresh]);

  if (!isAuthenticated) {
    return (
      <aside className={styles.sidebar}>
        <div className={styles.header}>
          <h2 className={styles.title}>Chat Sessions</h2>
        </div>
        <div className={styles.emptyState}>
          <p>Sign in to view your chat history</p>
        </div>
      </aside>
    );
  }

  return (
    <aside className={styles.sidebar}>
      <div className={styles.header}>
        <h2 className={styles.title}>Sessions</h2>
        <button
          className={styles.newButton}
          onClick={handleNewSession}
          disabled={isCreating}
          aria-label="New chat"
        >
          {isCreating ? (
            <span className={styles.spinner} />
          ) : (
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <line x1="12" y1="5" x2="12" y2="19" />
              <line x1="5" y1="12" x2="19" y2="12" />
            </svg>
          )}
        </button>
      </div>

      {error && (
        <div className={styles.error}>
          {error}
          <button onClick={() => setError(null)} className={styles.dismissError}>
            &times;
          </button>
        </div>
      )}

      <div className={styles.sessionList}>
        {isLoading ? (
          <div className={styles.loading}>
            <span className={styles.spinner} />
            <span>Loading sessions...</span>
          </div>
        ) : sessions.length === 0 ? (
          <div className={styles.emptyState}>
            <p>No chat sessions yet</p>
            <p className={styles.emptyHint}>Start a new conversation!</p>
          </div>
        ) : (
          sessions.map((session) => (
            <div
              key={session.id}
              className={`${styles.sessionItem} ${
                activeSessionId === session.id ? styles.active : ''
              } ${deletingId === session.id ? styles.deleting : ''}`}
            >
              <button
                className={styles.sessionButton}
                onClick={() => onSessionSelect(session.id)}
                disabled={deletingId === session.id}
              >
                <span className={styles.sessionTitle}>
                  {session.title || 'New conversation'}
                </span>
                <span className={styles.sessionMeta}>
                  <span className={styles.sessionTime}>
                    {formatRelativeTime(session.last_interaction)}
                  </span>
                  {session.message_count > 0 && (
                    <span className={styles.sessionCount}>
                      {session.message_count} msg{session.message_count !== 1 ? 's' : ''}
                    </span>
                  )}
                </span>
              </button>

              {showDeleteConfirm === session.id ? (
                <div className={styles.deleteConfirm}>
                  <button
                    className={styles.confirmYes}
                    onClick={() => handleDelete(session.id)}
                    disabled={deletingId === session.id}
                  >
                    Delete
                  </button>
                  <button
                    className={styles.confirmNo}
                    onClick={() => setShowDeleteConfirm(null)}
                  >
                    Cancel
                  </button>
                </div>
              ) : (
                <button
                  className={styles.deleteButton}
                  onClick={(e) => {
                    e.stopPropagation();
                    setShowDeleteConfirm(session.id);
                  }}
                  disabled={deletingId === session.id}
                  aria-label="Delete session"
                >
                  <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                    <polyline points="3 6 5 6 21 6" />
                    <path d="M19 6v14a2 2 0 0 1-2 2H7a2 2 0 0 1-2-2V6m3 0V4a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v2" />
                  </svg>
                </button>
              )}
            </div>
          ))
        )}
      </div>
    </aside>
  );
};

export default SessionSidebar;
