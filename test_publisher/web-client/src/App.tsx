import React from 'react';
import './App.css';
import { useROS } from './hooks/useROS';

function App() {
  const { connected, messageCount, lastMessage, error } = useROS();

  return (
    <div className="App">
      <header className="App-header">
        <h1>ROS2 Random Data Viewer</h1>
        
        {/* Connection Status */}
        <div className="status-container">
          <div className={`status-indicator ${connected ? 'connected' : 'disconnected'}`}>
            {connected ? 'Connected' : 'Disconnected'}
          </div>
          {error && <div className="error-message">{error}</div>}
        </div>

        {/* Message Counter */}
        <div className="counter-container">
          <h2>Messages Received:</h2>
          <div className="counter">{messageCount}</div>
        </div>

        {/* Last Message */}
        <div className="message-container">
          <h2>Last Message:</h2>
          {lastMessage ? (
            <div className="message-content">
              <p><strong>Random Number:</strong> {lastMessage.random_number}</p>
              <p><strong>Message:</strong> {lastMessage.message}</p>
              <p><strong>Timestamp:</strong> {lastMessage.timestamp}</p>
            </div>
          ) : (
            <p>Waiting for messages...</p>
          )}
        </div>
      </header>
    </div>
  );
}

export default App;