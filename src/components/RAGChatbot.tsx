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
    // In a real implementation, we would fetch and index all document content
    // For this implementation, we'll create a comprehensive index from the existing docs
    const miniSearch = new MiniSearch({
      fields: ['title', 'content'],
      storeFields: ['title', 'content', 'url'],
      searchOptions: {
        prefix: true,
        fuzzy: 0.2,
        boost: { title: 1.5, content: 1 }
      }
    });

    // This would normally be populated dynamically from all docs
    // For now, we'll use a more comprehensive set of sample content
    const docs: Document[] = [
      {
        id: 'intro',
        title: 'Introduction to Physical AI & Humanoid Robotics',
        content: 'Physical AI represents a paradigm shift in artificial intelligence research, focusing on creating embodied systems that interact with the physical world. Unlike traditional AI that operates primarily in digital spaces, Physical AI systems must understand and manipulate the physical environment through sensors, actuators, and sophisticated control mechanisms. This textbook covers the fundamentals of Physical AI and humanoid robotics, providing students with the knowledge needed to develop intelligent robotic systems.',
        url: '/docs/intro'
      },
      {
        id: 'digital-twin-intro',
        title: 'Introduction to Digital Twins in Robotics',
        content: 'A digital twin in robotics is a virtual replica of a physical robotic system that exists simultaneously in the digital space. This concept has become increasingly important in robotics development as it allows engineers and researchers to test algorithms, validate designs, and develop control strategies in a safe, cost-effective virtual environment before deploying on physical hardware. Digital twins serve as a bridge between the physical and digital worlds, enabling real-time monitoring, simulation, and optimization of robotic systems.',
        url: '/docs/modules/digital-twin/intro'
      },
      {
        id: 'gazebo-installation',
        title: 'Gazebo Installation and Configuration',
        content: 'Gazebo is a physics-based simulation environment that enables accurate and efficient testing of robotics algorithms, designs, and scenarios. This chapter provides detailed instructions for installing and configuring Gazebo with ROS2 integration, which is essential for the digital twin module. Before installing Gazebo, ensure your system meets the requirements including a multi-core processor, 8GB+ RAM, and OpenGL 2.1 compatible GPU. The installation process involves several steps including system preparation, dependency installation, and configuration verification.',
        url: '/docs/modules/digital-twin/gazebo-installation'
      },
      {
        id: 'gazebo-robot-modeling',
        title: 'Robot Modeling in Gazebo',
        content: 'Robot modeling in Gazebo involves creating accurate 3D representations of physical robots that can be simulated in virtual environments. This chapter covers the fundamentals of robot modeling, including the Unified Robot Description Format (URDF), joint configurations, physical properties, and sensor integration. URDF (Unified Robot Description Format) is an XML-based format used to describe robots in ROS and Gazebo. It defines the robot\'s physical structure, including links, joints, and other properties.',
        url: '/docs/modules/digital-twin/gazebo-robot-modeling'
      },
      {
        id: 'gazebo-simulation-basics',
        title: 'Simulation Basics in Gazebo',
        content: 'This chapter covers the fundamental concepts of running simulations in Gazebo. You\'ll learn how to create and configure simulation environments, run basic simulations, and understand the core components that make up a Gazebo simulation. World files define the complete simulation environment, including models (robots, objects), physics properties, lighting and environment settings, and plugins and controllers.',
        url: '/docs/modules/digital-twin/gazebo-simulation-basics'
      },
      {
        id: 'gazebo-physics-engines',
        title: 'Physics Engines in Gazebo',
        content: 'Physics engines are the core components that simulate the physical behavior of objects in Gazebo. They calculate forces, collisions, and movements to create realistic interactions between objects in the simulation environment. This chapter covers the different physics engines available in Gazebo, their characteristics, and how to configure them for optimal performance. Gazebo supports multiple physics engines including ODE (Open Dynamics Engine), Bullet Physics, and Simbody.',
        url: '/docs/modules/digital-twin/gazebo-physics-engines'
      },
      {
        id: 'unity-integration',
        title: 'Unity Integration for Robotics',
        content: 'Unity is a powerful 3D development platform that can be integrated with robotics workflows to create realistic visualizations, human-robot interaction interfaces, and virtual environments. This chapter covers how to integrate Unity with robotics systems, particularly in the context of digital twin implementations. Unity Robotics Hub provides robotics-specific tools and packages including ROS-TCP-Connector, which enables communication between Unity and ROS/ROS2.',
        url: '/docs/modules/digital-twin/unity-integration'
      },
      {
        id: 'practical-examples',
        title: 'Practical Examples in Digital Twin Robotics',
        content: 'This chapter provides practical examples that demonstrate the integration of all concepts covered in the Digital Twin module. These examples will help you apply Gazebo simulation, Unity integration, and physics engines in real-world scenarios. The examples include mobile robot navigation in Gazebo, Unity visualization of Gazebo simulation, multi-robot coordination, and physics simulation comparison.',
        url: '/docs/modules/digital-twin/practical-examples'
      }
    ];

    miniSearch.addAll(docs);
    setSearchIndex(miniSearch);
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
      const results = searchIndex.search(userMessage);
      
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
        // Optionally show a tooltip or context menu
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
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
          backgroundColor: '#3578e5',
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
            backgroundColor: 'white',
            border: '1px solid #ccc',
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
              backgroundColor: '#3578e5',
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
              backgroundColor: '#f9f9f9'
            }}
          >
            {messages.length === 0 ? (
              <div style={{ textAlign: 'center', color: '#666', marginTop: '20px' }}>
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
                        backgroundColor: message.isUser ? '#e3e6eb' : '#3578e5',
                        color: message.isUser ? '#000' : '#fff',
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
                                style={{ color: '#3578e5', textDecoration: 'none' }}
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
                        backgroundColor: '#3578e5',
                        color: '#fff',
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
              borderTop: '1px solid #eee',
              backgroundColor: 'white'
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
                  border: '1px solid #ccc',
                  borderRadius: '18px',
                  marginRight: '8px'
                }}
              />
              <button
                onClick={handleSendMessage}
                disabled={isLoading || !inputValue.trim()}
                style={{
                  padding: '8px 16px',
                  backgroundColor: '#3578e5',
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
            <div style={{ fontSize: '12px', color: '#666', marginTop: '8px', textAlign: 'center' }}>
              <button 
                onClick={handleSelectedText}
                style={{ 
                  background: 'none', 
                  border: 'none', 
                  color: '#3578e5', 
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