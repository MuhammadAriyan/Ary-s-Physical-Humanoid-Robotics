import React, { useState, useEffect } from 'react';
import { useAuth } from '../components/Auth';
import { getSession } from '../lib/auth-client';
import '../styles/ChatPage.css';

interface Message {
  id: string;
  sender: 'user' | 'fubuni';
  content: string;
  timestamp: string;
}

interface ChatSession {
  id: string;
  title: string;
  lastInteraction: string;
}

const ChatPage: React.FC = () => {
  const { isAuthenticated, user, isLoading: authLoading } = useAuth();
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [sessions, setSessions] = useState<ChatSession[]>([]);
  const [activeSession, setActiveSession] = useState<string | null>(null);
  const [isLoading, setIsLoading] = useState(false);
  const [isSidebarOpen, setIsSidebarOpen] = useState(true);

  // Load user's chat sessions
  useEffect(() => {
    if (isAuthenticated && !authLoading) {
      fetchChatSessions();
    }
  }, [isAuthenticated, authLoading]);

  const fetchChatSessions = async () => {
    if (!isAuthenticated) return;
    
    try {
      const session = await getSession();
      const token = session?.data?.session?.token;
      
      if (!token) {
        alert('Authentication required');
        return;
      }
      
      const response = await fetch('http://localhost:8000/api/chat/sessions', {
        headers: {
          'Authorization': `Bearer ${token}`,
        },
      });
      
      if (response.ok) {
        const data = await response.json();
        setSessions(data);
      } else {
        console.error('Failed to fetch chat sessions');
      }
    } catch (error) {
      console.error('Error fetching sessions:', error);
    }
  };

  const fetchChatHistory = async (sessionId: string) => {
    try {
      const session = await getSession();
      const token = session?.data?.session?.token;
      
      if (!token) {
        alert('Authentication required');
        return;
      }
      
      const response = await fetch(`http://localhost:8000/api/chat/history/${sessionId}`, {
        headers: {
          'Authorization': `Bearer ${token}`,
        },
      });
      
      if (response.ok) {
        const data = await response.json();
        // Convert to Message format
        const chatMessages: Message[] = data.messages.map((msg: any) => ({
          id: msg.id || `msg-${Date.now()}`,
          sender: msg.sender,
          content: msg.content,
          timestamp: msg.timestamp || new Date().toISOString(),
        }));
        setMessages(chatMessages);
        setActiveSession(sessionId);
      } else {
        console.error('Failed to fetch chat history');
      }
    } catch (error) {
      console.error('Error fetching chat history:', error);
    }
  };

  const createNewSession = () => {
    setActiveSession(null);
    setMessages([]);
    setInputValue('');
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading || !isAuthenticated) return;

    setIsLoading(true);

    try {
      const session = await getSession();
      const token = session?.data?.session?.token;

      if (!token) {
        alert('Authentication required');
        return;
      }

      // Add user message optimistically
      const userMessage: Message = {
        id: `user-${Date.now()}`,
        sender: 'user',
        content: inputValue,
        timestamp: new Date().toISOString(),
      };
      
      setMessages(prev => [...prev, userMessage]);
      setInputValue('');

      // Send to backend
      const response = await fetch('http://localhost:8000/api/chat/stream', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`,
        },
        body: JSON.stringify({
          message: inputValue,
          session_id: activeSession || undefined,
        }),
      });

      if (response.status === 401) {
        alert('Session expired. Please sign in again.');
        return;
      }
      
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      // Handle streaming response
      const reader = response.body?.getReader();
      const decoder = new TextDecoder();
      let fubuniMessage: Message = {
        id: `fubuni-${Date.now()}`,
        sender: 'fubuni',
        content: '',
        timestamp: new Date().toISOString(),
      };

      if (reader) {
        setMessages(prev => [...prev, fubuniMessage]);
        
        let done = false;
        while (!done) {
          const { value, done: readerDone } = await reader.read();
          done = readerDone;
          
          if (value) {
            const chunk = decoder.decode(value, { stream: true });
            // Process Server-Sent Events
            const lines = chunk.split('\n');
            for (const line of lines) {
              if (line.startsWith('data: ')) {
                const data = line.slice(6);
                if (data === '[DONE]') {
                  done = true;
                  break;
                }
                try {
                  const parsed = JSON.parse(data);
                  if (parsed.type === 'text') {
                    fubuniMessage.content += parsed.content;
                    // Update the last message in the state
                    setMessages(prev => {
                      const newPrev = [...prev];
                      newPrev[newPrev.length - 1] = fubuniMessage;
                      return newPrev;
                    });
                  }
                } catch (e) {
                  // Ignore parsing errors for other event types
                }
              }
            }
          }
        }
        
        reader.releaseLock();
      }

      // Refresh sessions to show the updated one
      setTimeout(() => fetchChatSessions(), 500);
      
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage: Message = {
        id: `error-${Date.now()}`,
        sender: 'fubuni',
        content: 'Sorry, I encountered an error processing your request. Please try again.',
        timestamp: new Date().toISOString(),
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  if (authLoading) {
    return <div className="chat-page">Loading...</div>;
  }

  if (!isAuthenticated) {
    return (
      <div className="chat-page">
        <div className="auth-required">
          <h2>Please sign in to access the chat</h2>
          <p>Authentication is required to access your chat history and start new conversations.</p>
        </div>
      </div>
    );
  }

  return (
    <div className="chat-page">
      {/* Sidebar */}
      <div className={`chat-sidebar ${isSidebarOpen ? 'open' : 'closed'}`}>
        <div className="sidebar-header">
          <h2>Chat Sessions</h2>
          <button onClick={createNewSession} className="new-session-btn">
            + New Chat
          </button>
        </div>
        
        <div className="sessions-list">
          {sessions.map(session => (
            <div
              key={session.id}
              className={`session-item ${activeSession === session.id ? 'active' : ''}`}
              onClick={() => fetchChatHistory(session.id)}
            >
              <div className="session-title">{session.title}</div>
              <div className="session-date">{new Date(session.lastInteraction).toLocaleDateString()}</div>
            </div>
          ))}
          
          {sessions.length === 0 && (
            <div className="no-sessions">No chat sessions yet</div>
          )}
        </div>
      </div>
      
      {/* Main Chat Area */}
      <div className="chat-main">
        <div className="chat-messages">
          {messages.map(message => (
            <div key={message.id} className={`message ${message.sender}`}>
              <div className="message-content">{message.content}</div>
              <div className="message-timestamp">
                {new Date(message.timestamp).toLocaleTimeString()}
              </div>
            </div>
          ))}
        </div>
        
        <form onSubmit={handleSubmit} className="chat-input-form">
          <input
            type="text"
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            placeholder="Type your message..."
            disabled={isLoading}
            className="chat-input"
          />
          <button type="submit" disabled={isLoading} className="send-button">
            {isLoading ? 'Sending...' : 'Send'}
          </button>
        </form>
      </div>
    </div>
  );
};

export default ChatPage;