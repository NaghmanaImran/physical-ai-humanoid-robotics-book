import React, { useState, useEffect, useRef } from 'react';
import MiniSearch from 'minisearch';

interface Document {
  id: string;
  title: string;
  content: string;
  url: string;
}

const RAGChatbot: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<{id: number; text: string; isUser: boolean; sources?: string[]}[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [searchIndex, setSearchIndex] = useState<MiniSearch | null>(null);

  const messagesEndRef = useRef<null | HTMLDivElement>(null);
  const inputRef = useRef<HTMLInputElement>(null);

  // Initialize search index with all docs content
  useEffect(() => {
    const initializeSearch = async () => {
      // Create MiniSearch instance
      const miniSearch = new MiniSearch({
        fields: ['title', 'content'],
        storeFields: ['title', 'content', 'url'],
        searchOptions: {
          prefix: true,
          fuzzy: 0.2,
          boost: { title: 1.5, content: 1 }
        }
      });

      try {
        // Dynamically import the generated documentation index
        const docsIndex = await import('../utils/docs-index.json');
        const docs = docsIndex.default || docsIndex;

        // Add all documents to the search index
        miniSearch.addAll(docs);
        setSearchIndex(miniSearch);
      } catch (error) {
        console.error('Error initializing search index:', error);
      }
    };

    initializeSearch();
  }, []);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const toggleChat = () => {
    setIsOpen(!isOpen);
    if (!isOpen && inputRef.current) {
      setTimeout(() => inputRef.current?.focus(), 100);
    }
  };

  const handleSendMessage = async () => {
    if (!inputValue.trim() || !searchIndex || isLoading) return;

    const userMessage = inputValue.trim();
    setInputValue('');

    // Add user message
    const userMsgId = Date.now();
    setMessages(prev => [...prev, { id: userMsgId, text: userMessage, isUser: true }]);

    setIsLoading(true);

    try {
      // Search for relevant documents
      const results = searchIndex.search(userMessage, {
        prefix: true,
        fuzzy: 0.2,
        boost: { title: 1.5, content: 1 }
      });

      // Get top 3 results
      const topResults = results.slice(0, 3);

      // Create context from top results
      const context = topResults.map(result =>
        `Title: ${result.title}\nContent: ${result.content.substring(0, 500)}`
      ).join('\n\n');

      // Generate response based on context
      let responseText = '';
      if (topResults.length > 0) {
        responseText = `Based on the textbook content, here's what I found:\n\n${context.substring(0, 1000)}...`;
      } else {
        responseText = "I couldn't find specific information about this topic in the textbook. Please try rephrasing your question or check other chapters.";
      }

      // Extract sources
      const sources = topResults.map(result => result.url);

      // Add bot response
      setMessages(prev => [...prev, {
        id: Date.now(),
        text: responseText,
        isUser: false,
        sources
      }]);
    } catch (error) {
      console.error('Error processing query:', error);
      setMessages(prev => [...prev, {
        id: Date.now(),
        text: 'Sorry, I encountered an error processing your query. Please try again.',
        isUser: false
      }]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyDown = (e: React.KeyboardEvent<HTMLInputElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  const handleSelectedText = () => {
    const selectedText = window.getSelection()?.toString().trim();
    if (selectedText) {
      setInputValue(selectedText);
      setIsOpen(true);
      setTimeout(() => inputRef.current?.focus(), 100);
    }
  };

  // Listen for text selection
  useEffect(() => {
    const handleSelection = () => {
      const selectedText = window.getSelection()?.toString().trim();
      if (selectedText && selectedText.length > 10) { // Only show for meaningful selections
        // Show a tooltip or context menu to ask about selected text
        const selection = window.getSelection();
        if (selection && selection.rangeCount > 0) {
          const range = selection.getRangeAt(0);
          const rect = range.getBoundingClientRect();

          // Create a temporary button element
          const button = document.createElement('div');
          button.id = 'ask-about-text-button';
          button.textContent = 'Ask about this text';
          button.style.position = 'absolute';
          button.style.top = `${rect.top - 30}px`;
          button.style.left = `${rect.left}px`;
          button.style.backgroundColor = '#3578e5';
          button.style.color = 'white';
          button.style.padding = '5px 10px';
          button.style.borderRadius = '4px';
          button.style.cursor = 'pointer';
          button.style.fontSize = '12px';
          button.style.zIndex = '9999';
          button.style.boxShadow = '0 2px 4px rgba(0,0,0,0.2)';

          button.onclick = () => {
            document.body.removeChild(button);
            handleSelectedText();
          };

          document.body.appendChild(button);

          // Remove the button after a delay
          setTimeout(() => {
            if (document.contains(button)) {
              document.body.removeChild(button);
            }
          }, 3000);
        }
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
      // Clean up any remaining button
      const existingButton = document.getElementById('ask-about-text-button');
      if (existingButton) {
        document.body.removeChild(existingButton);
      }
    };
  }, []);

  return (
    <>
      {/* Floating chat button */}
      <button
        onClick={toggleChat}
        className="rag-chatbot-button"
        style={{
          position: 'fixed',
          bottom: '20px',
          right: '20px',
          width: '60px',
          height: '60px',
          borderRadius: '50%',
          backgroundColor: 'var(--ifm-color-primary)',
          color: 'white',
          border: 'none',
          fontSize: '24px',
          cursor: 'pointer',
          zIndex: 1000,
          boxShadow: '0 4px 8px rgba(0,0,0,0.2)',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center'
        }}
      >
        💬
      </button>

      {/* Chat window */}
      {isOpen && (
        <div
          className="rag-chatbot-window"
          style={{
            position: 'fixed',
            bottom: '90px',
            right: '20px',
            width: '400px',
            height: '500px',
            backgroundColor: 'var(--ifm-background-surface-color)',
            border: '1px solid var(--ifm-color-emphasis-300)',
            borderRadius: '8px',
            boxShadow: '0 4px 12px rgba(0,0,0,0.15)',
            zIndex: 1000,
            display: 'flex',
            flexDirection: 'column',
            fontFamily: 'system-ui, -apple-system, sans-serif'
          }}
        >
          {/* Header */}
          <div
            style={{
              backgroundColor: 'var(--ifm-color-primary)',
              color: 'white',
              padding: '12px',
              borderTopLeftRadius: '8px',
              borderTopRightRadius: '8px',
              display: 'flex',
              justifyContent: 'space-between',
              alignItems: 'center'
            }}
          >
            <h3 style={{ margin: 0, fontSize: '16px' }}>Textbook Assistant</h3>
            <button
              onClick={toggleChat}
              style={{
                background: 'none',
                border: 'none',
                color: 'white',
                fontSize: '18px',
                cursor: 'pointer'
              }}
            >
              ×
            </button>
          </div>

          {/* Messages container */}
          <div
            style={{
              flex: 1,
              padding: '16px',
              overflowY: 'auto',
              backgroundColor: 'var(--ifm-background-color)'
            }}
          >
            {messages.length === 0 ? (
              <div style={{ textAlign: 'center', color: 'var(--ifm-color-emphasis-700)', marginTop: '20px' }}>
                Ask me anything about the Physical AI & Humanoid Robotics textbook!
              </div>
            ) : (
              <div>
                {messages.map((message) => (
                  <div
                    key={message.id}
                    style={{
                      marginBottom: '12px',
                      textAlign: message.isUser ? 'right' : 'left'
                    }}
                  >
                    <div
                      style={{
                        display: 'inline-block',
                        padding: '8px 12px',
                        borderRadius: '18px',
                        backgroundColor: message.isUser
                          ? 'var(--ifm-color-emphasis-200)'
                          : 'var(--ifm-color-primary)',
                        color: message.isUser ? 'var(--ifm-font-color-base)' : 'white',
                        maxWidth: '80%',
                        wordWrap: 'break-word'
                      }}
                    >
                      {message.text.split('\n').map((line, i) => (
                        <div key={i}>{line}</div>
                      ))}
                    </div>

                    {!message.isUser && message.sources && message.sources.length > 0 && (
                      <div style={{ fontSize: '12px', marginTop: '4px' }}>
                        <strong>Sources:</strong>
                        <ul style={{ margin: '4px 0', paddingLeft: '20px' }}>
                          {message.sources.map((source, idx) => (
                            <li key={idx}>
                              <a
                                href={source}
                                target="_blank"
                                rel="noopener noreferrer"
                                style={{
                                  color: 'var(--ifm-color-primary)',
                                  textDecoration: 'none'
                                }}
                              >
                                {source}
                              </a>
                            </li>
                          ))}
                        </ul>
                      </div>
                    )}
                  </div>
                ))}
                {isLoading && (
                  <div style={{ textAlign: 'left' }}>
                    <div
                      style={{
                        display: 'inline-block',
                        padding: '8px 12px',
                        borderRadius: '18px',
                        backgroundColor: 'var(--ifm-color-primary)',
                        color: 'white',
                        maxWidth: '80%'
                      }}
                    >
                      Thinking...
                    </div>
                  </div>
                )}
                <div ref={messagesEndRef} />
              </div>
            )}
          </div>

          {/* Input area */}
          <div
            style={{
              padding: '12px',
              borderTop: '1px solid var(--ifm-color-emphasis-300)',
              backgroundColor: 'var(--ifm-background-color)'
            }}
          >
            <div style={{ display: 'flex' }}>
              <input
                ref={inputRef}
                type="text"
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                onKeyDown={handleKeyDown}
                placeholder="Ask about the textbook..."
                disabled={isLoading}
                style={{
                  flex: 1,
                  padding: '8px 12px',
                  border: '1px solid var(--ifm-color-emphasis-300)',
                  borderRadius: '18px',
                  marginRight: '8px',
                  backgroundColor: 'var(--ifm-background-surface-color)',
                  color: 'var(--ifm-font-color-base)'
                }}
              />
              <button
                onClick={handleSendMessage}
                disabled={isLoading || !inputValue.trim()}
                style={{
                  padding: '8px 16px',
                  backgroundColor: 'var(--ifm-color-primary)',
                  color: 'white',
                  border: 'none',
                  borderRadius: '18px',
                  cursor: 'pointer',
                  opacity: (isLoading || !inputValue.trim()) ? 0.6 : 1
                }}
              >
                Send
              </button>
            </div>
            <div style={{ fontSize: '12px', color: 'var(--ifm-color-emphasis-700)', marginTop: '8px', textAlign: 'center' }}>
              <button
                onClick={handleSelectedText}
                style={{
                  background: 'none',
                  border: 'none',
                  color: 'var(--ifm-color-primary)',
                  textDecoration: 'underline',
                  cursor: 'pointer',
                  padding: 0
                }}
              >
                Ask about selected text
              </button>
            </div>
          </div>
        </div>
      )}
    </>
  );
};

export default RAGChatbot;