import React, { useState, useRef, useEffect } from 'react';
import styles from './ChatbotWidget.module.css';

const ChatbotWidget: React.FC = () => {
    const [isOpen, setIsOpen] = useState(false);
    const [messages, setMessages] = useState<{ text: string; sender: 'user' | 'bot' }[]>([]);
    const [inputMessage, setInputMessage] = useState('');
    const messagesEndRef = useRef<HTMLDivElement>(null);

    const toggleChat = () => {
        setIsOpen(!isOpen);
    };

    const scrollToBottom = () => {
        messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
    };

    useEffect(() => {
        scrollToBottom();
    }, [messages]);

    const handleSendMessage = async (e: React.FormEvent) => {
        e.preventDefault();
        if (inputMessage.trim() === '') return;

        const newUserMessage = { text: inputMessage, sender: 'user' as const };
        setMessages((prevMessages) => [...prevMessages, newUserMessage]);
        setInputMessage('');

        try {
            const response = await fetch('http://localhost:8000/api/chat', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({ query: inputMessage }),
            });

            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }

            const data = await response.json();
            setMessages((prevMessages) => [...prevMessages, { text: data.answer, sender: 'bot' }]);
        } catch (error) {
            console.error('Error sending message:', error);
            setMessages((prevMessages) => [
                ...prevMessages,
                { text: 'Sorry, something went wrong. Please try again later.', sender: 'bot' },
            ]);
        }
    };

    return (
        <>
            <button className={styles.chatButton} onClick={toggleChat}>
                {isOpen ? 'X' : 'Chat'}
            </button>

            {isOpen && (
                <div className={styles.chatPanel}>
                    <div className={styles.chatHeader}>
                        <h3>Book Chatbot</h3>
                        <button onClick={toggleChat}>X</button>
                    </div>
                    <div className={styles.chatMessages}>
                        {messages.map((msg, index) => (
                            <div key={index} className={`${styles.chatBubble} ${styles[msg.sender]}`}>
                                {msg.text}
                            </div>
                        ))}
                        <div ref={messagesEndRef} />
                    </div>
                    <form onSubmit={handleSendMessage} className={styles.chatInputForm}>
                        <input
                            type="text"
                            value={inputMessage}
                            onChange={(e) => setInputMessage(e.target.value)}
                            placeholder="Type your message..."
                            className={styles.chatInput}
                        />
                        <button type="submit" className={styles.sendButton}>
                            Send
                        </button>
                    </form>
                </div>
            )}
        </>
    );
};

export default ChatbotWidget;
