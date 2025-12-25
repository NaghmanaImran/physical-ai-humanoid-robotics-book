import React, { useState, useEffect, useRef } from 'react';
import MiniSearch from 'minisearch';

const SearchModal = ({ isOpen, onClose, searchResults, onSearch, query }) => {
  if (!isOpen) return null;

  return (
    <div className="search-modal-overlay" onClick={onClose}>
      <div className="search-modal-content" onClick={(e) => e.stopPropagation()}>
        <div className="search-modal-header">
          <input
            type="text"
            className="search-modal-input"
            placeholder="Search the textbook..."
            value={query}
            onChange={(e) => onSearch(e.target.value)}
            autoFocus
          />
          <button className="search-modal-close" onClick={onClose}>×</button>
        </div>
        <div className="search-modal-results">
          {searchResults.map((result, index) => (
            <div key={result.id} className="search-result-item">
              <a href={result.url} onClick={onClose}>
                <h4>{result.title}</h4>
                <p>{result.excerpt}</p>
              </a>
            </div>
          ))}
          {searchResults.length === 0 && query && (
            <div className="no-results">No results found for "{query}"</div>
          )}
        </div>
      </div>
    </div>
  );
};

export default function SearchButton() {
  const [isModalOpen, setIsModalOpen] = useState(false);
  const [searchResults, setSearchResults] = useState([]);
  const [searchQuery, setSearchQuery] = useState('');
  const [searchEngine, setSearchEngine] = useState(null);
  const [allDocs, setAllDocs] = useState([]);
  const inputRef = useRef(null);

  // Initialize search engine with all docs
  useEffect(() => {
    // In a real implementation, this would fetch all document content
    // For now, we'll simulate with sample data
    const docs = [
      { id: 'intro', title: 'Introduction to Physical AI & Humanoid Robotics', content: 'Physical AI and Humanoid Robotics textbook introduction', url: '/docs/intro' },
      { id: 'ros2-intro', title: 'Introduction to ROS2', content: 'Robot Operating System 2 for robotics applications', url: '/docs/ros2/intro' },
      { id: 'gazebo-intro', title: 'Introduction to Gazebo for Humanoid Robotics', content: 'Gazebo simulation environment for robotics', url: '/docs/gazebo/intro' },
      { id: 'isaac-intro', title: 'Introduction to NVIDIA Isaac for Humanoid Robotics', content: 'NVIDIA Isaac platform for AI-powered robotics', url: '/docs/nvidia-isaac/intro' },
      { id: 'vla-intro', title: 'Introduction to VLA for Humanoid Robotics', content: 'Vision-Language-Action models for robotics', url: '/docs/vla/intro' },
      { id: 'capstone', title: 'Capstone Project: Autonomous Humanoid Robot', content: 'Complete project integrating all modules', url: '/docs/capstone-project' }
    ];

    setAllDocs(docs);

    const miniSearch = new MiniSearch({
      fields: ['title', 'content'],
      storeFields: ['title', 'content', 'url'],
      searchOptions: {
        boost: { title: 2 },
        fuzzy: 0.2
      }
    });

    miniSearch.addAll(docs.map((doc, index) => ({
      id: index,
      title: doc.title,
      content: doc.content,
      url: doc.url
    })));

    setSearchEngine(miniSearch);
  }, []);

  const handleSearch = (query) => {
    setSearchQuery(query);
    if (searchEngine && query.trim()) {
      const results = searchEngine.search(query, { prefix: true });
      setSearchResults(results.map(result => {
        const originalDoc = allDocs[result.id];
        return {
          id: result.id,
          title: originalDoc.title,
          content: originalDoc.content,
          url: originalDoc.url,
          excerpt: originalDoc.content.substring(0, 100) + '...'
        };
      }));
    } else {
      setSearchResults([]);
    }
  };

  const toggleModal = () => {
    setIsModalOpen(!isModalOpen);
  };

  const closeModal = () => {
    setIsModalOpen(false);
    setSearchQuery('');
    setSearchResults([]);
  };

  // Handle keyboard shortcut (Ctrl/Cmd+K)
  useEffect(() => {
    const handleKeyDown = (e) => {
      if ((e.ctrlKey || e.metaKey) && e.key === 'k') {
        e.preventDefault();
        toggleModal();
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, []);

  return (
    <div>
      <button 
        className="floating-search-button" 
        onClick={toggleModal}
        title="Search (Ctrl/Cmd+K)"
      >
        🔍
      </button>
      
      <SearchModal 
        isOpen={isModalOpen} 
        onClose={closeModal} 
        searchResults={searchResults}
        onSearch={handleSearch}
        query={searchQuery}
      />
      
      <style jsx>{`
        .floating-search-button {
          position: fixed;
          bottom: 20px;
          right: 20px;
          width: 60px;
          height: 60px;
          border-radius: 50%;
          background-color: #25c2a0;
          color: white;
          border: none;
          font-size: 24px;
          cursor: pointer;
          box-shadow: 0 4px 12px rgba(0,0,0,0.15);
          z-index: 1000;
          display: flex;
          align-items: center;
          justify-content: center;
        }
        
        .floating-search-button:hover {
          background-color: #21af90;
          transform: scale(1.05);
        }
        
        .search-modal-overlay {
          position: fixed;
          top: 0;
          left: 0;
          right: 0;
          bottom: 0;
          background-color: rgba(0, 0, 0, 0.7);
          z-index: 1001;
          display: flex;
          justify-content: center;
          align-items: flex-start;
          padding-top: 100px;
        }
        
        .search-modal-content {
          width: 90%;
          max-width: 600px;
          background: white;
          border-radius: 8px;
          overflow: hidden;
        }
        
        .search-modal-header {
          display: flex;
          align-items: center;
          padding: 15px;
          border-bottom: 1px solid #eee;
          background: white;
        }
        
        .search-modal-input {
          flex: 1;
          padding: 10px;
          border: 1px solid #ccc;
          border-radius: 4px;
          font-size: 16px;
        }
        
        .search-modal-close {
          background: none;
          border: none;
          font-size: 24px;
          cursor: pointer;
          margin-left: 10px;
          color: #999;
        }
        
        .search-modal-close:hover {
          color: #333;
        }
        
        .search-modal-results {
          max-height: 70vh;
          overflow-y: auto;
          padding: 10px;
        }
        
        .search-result-item {
          padding: 10px 0;
          border-bottom: 1px solid #eee;
        }
        
        .search-result-item:last-child {
          border-bottom: none;
        }
        
        .search-result-item a {
          text-decoration: none;
          color: inherit;
        }
        
        .search-result-item h4 {
          margin: 0 0 5px 0;
          color: #25c2a0;
        }
        
        .search-result-item p {
          margin: 0;
          color: #666;
        }
        
        .no-results {
          padding: 20px;
          text-align: center;
          color: #999;
        }
      `}</style>
    </div>
  );
}