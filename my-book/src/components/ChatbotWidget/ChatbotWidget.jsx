import React, { useState, useEffect, useRef } from 'react';
import styles from './ChatbotWidget.module.css';

export default function ChatbotWidget() {
  const [messages, setMessages] = useState([]);
  const [question, setQuestion] = useState('');
  const [isConnected, setIsConnected] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId, setSessionId] = useState(null);
  const [selectedText, setSelectedText] = useState(null);
  const [isExpanded, setIsExpanded] = useState(true);
  const messagesEndRef = useRef(null);

  const API_BASE_URL = 'https://saimawaheedsaima5-backend-deploy.hf.space';
  const BOOK_ID = 'my-book';

  // Initialize chatbot on mount
  useEffect(() => {
    initializeChatbot();
  }, []);

  // Scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Test connection and create session
  const initializeChatbot = async () => {
    try {
      const response = await fetch(`${API_BASE_URL}/health`);
      if (response.ok) {
        setIsConnected(true);
        addSystemMessage('✅ Connected to chatbot backend');
        await createSession();
      } else {
        throw new Error('Backend not responding');
      }
    } catch (error) {
      console.error('❌ Connection error:', error);
      setIsConnected(false);
      addSystemMessage(
        '⚠️ Cannot connect to backend. Please ensure the server is running on http://localhost:8001'
      );
    }
  };

  // Create a session
  const createSession = async () => {
    try {
      const response = await fetch(`${API_BASE_URL}/session`, {
        method: 'POST',
      });

      if (response.ok) {
        const data = await response.json();
        setSessionId(data.session_id);
        addSystemMessage('✅ Ready to answer your questions!');
      }
    } catch (error) {
      console.error('❌ Session creation error:', error);
      addSystemMessage('❌ Failed to create session');
    }
  };

  // Add system message to chat
  const addSystemMessage = (message) => {
    setMessages((prev) => [...prev, { role: 'system', content: message }]);
  };

  // Send question to backend
  const handleSendQuestion = async () => {
    const trimmedQuestion = question.trim();

    if (!trimmedQuestion) {
      addSystemMessage('⚠️ Please enter a question');
      return;
    }

    if (!isConnected) {
      addSystemMessage('❌ Not connected to backend');
      return;
    }

    if (!sessionId) {
      addSystemMessage('❌ No active session');
      return;
    }

    // Add user message
    setMessages((prev) => [...prev, { role: 'user', content: trimmedQuestion }]);
    setQuestion('');
    setIsLoading(true);

    // Show typing indicator
    setMessages((prev) => [...prev, { role: 'bot', content: 'typing' }]);

    try {
      const requestBody = {
        question: trimmedQuestion,
        book_id: BOOK_ID,
        selected_text: selectedText || null,
        top_k: 5,
      };

      const response = await fetch(`${API_BASE_URL}/query`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(requestBody),
      });

      // Remove typing indicator
      setMessages((prev) => prev.filter((msg) => msg.content !== 'typing'));

      if (response.ok) {
        const data = await response.json();
        setMessages((prev) => [...prev, { role: 'bot', content: data, type: 'response' }]);
        setSelectedText(null); // Clear selected text after use
      } else {
        const error = await response.json();
        addSystemMessage(`❌ Error: ${error.detail}`);
      }
    } catch (error) {
      console.error('❌ Query error:', error);
      setMessages((prev) => prev.filter((msg) => msg.content !== 'typing'));
      addSystemMessage(`❌ Failed to get response: ${error.message}`);
    } finally {
      setIsLoading(false);
    }
  };

  // Handle key press
  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey && !isLoading) {
      e.preventDefault();
      handleSendQuestion();
    }
  };

  // Render a single message
  const renderMessage = (msg, index) => {
    if (msg.role === 'system') {
      return (
        <div key={index} className={styles.systemMessage}>
          {msg.content}
        </div>
      );
    }

    if (msg.role === 'user') {
      return (
        <div key={index} className={styles.userMessage}>
          <div className={styles.messageBubble}>{msg.content}</div>
        </div>
      );
    }

    if (msg.content === 'typing') {
      return (
        <div key={index} className={styles.botMessage}>
          <div className={styles.typingIndicator}>
            <span></span>
            <span></span>
            <span></span>
          </div>
        </div>
      );
    }

    if (msg.type === 'response' && msg.content) {
      const data = msg.content;
      return (
        <div key={index} className={styles.botMessage}>
          <div className={styles.messageBubble}>
            {data.is_fallback && (
              <div className={styles.fallbackBadge}>
                ⚠️ General Knowledge (not from book)
              </div>
            )}
            <div className={styles.answer}>{data.answer}</div>

            {data.sources && data.sources.length > 0 && (
              <div className={styles.sources}>
                <strong>📚 Sources ({data.sources.length}):</strong>
                {data.sources.map((source, idx) => {
                  const score = (source.similarity_score * 100).toFixed(0);
                  return (
                    <div key={idx} className={styles.sourceItem}>
                      <strong>Page {source.page_num}</strong> ({score}% match)
                      <br />
                      <em>"{source.chunk_text.substring(0, 120)}..."</em>
                    </div>
                  );
                })}
              </div>
            )}

            <div className={styles.metadata}>
              {data.model} | {data.latency_ms}ms
            </div>
          </div>
        </div>
      );
    }

    return null;
  };

  if (!isExpanded) {
    return (
      <button
        className={styles.minimizedButton}
        onClick={() => setIsExpanded(true)}
        title="Open chatbot"
      >
        💬
      </button>
    );
  }

  return (
    <div className={styles.chatbotWidget}>
      <div className={styles.header}>
        <h3>💬 Ask About This Book</h3>
        <div className={styles.controls}>
          <span className={`${styles.statusDot} ${isConnected ? styles.connected : ''}`}></span>
          <button
            className={styles.minimizeBtn}
            onClick={() => setIsExpanded(false)}
            title="Minimize"
          >
            −
          </button>
        </div>
      </div>

      <div className={styles.messagesContainer}>
        {messages.length === 0 && (
          <div className={styles.welcomeMessage}>
            <h4>Welcome! 👋</h4>
            <p>Ask any questions about the documentation. I'll search through the content and provide answers with citations.</p>
            <div className={styles.tips}>
              <p><strong>💡 Pro Tips:</strong></p>
              <ul>
                <li>Select text on the page to use it as context</li>
                <li>Ask follow-up questions naturally</li>
                <li>Questions are answered from book content when possible</li>
              </ul>
            </div>
          </div>
        )}
        {messages.map((msg, idx) => renderMessage(msg, idx))}
        <div ref={messagesEndRef} />
      </div>

      <div className={styles.inputArea}>
        <div className={styles.inputWrapper}>
          <input
            type="text"
            value={question}
            onChange={(e) => setQuestion(e.target.value)}
            onKeyPress={handleKeyPress}
            placeholder={isConnected ? 'Ask a question...' : 'Backend offline...'}
            disabled={!isConnected || isLoading}
            className={styles.input}
          />
          <button
            onClick={handleSendQuestion}
            disabled={!isConnected || isLoading}
            className={styles.sendBtn}
          >
            {isLoading ? '⏳' : '📤'}
          </button>
        </div>
        {selectedText && (
          <div className={styles.selectedInfo}>
            ✓ Using selected text ({selectedText.length} chars)
          </div>
        )}
      </div>
    </div>
  );
}
