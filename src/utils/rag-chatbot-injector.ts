import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

if (ExecutionEnvironment.canUseDOM) {
  // Dynamically import the RAGChatbot component and render it
  (async () => {
    const { default: RAGChatbot } = await import('../components/RAGChatbot');
    const React = await import('react');
    const ReactDOM = await import('react-dom/client');

    // Create a container for the chatbot
    let chatbotContainer = document.getElementById('rag-chatbot-root');
    if (!chatbotContainer) {
      chatbotContainer = document.createElement('div');
      chatbotContainer.id = 'rag-chatbot-root';
      document.body.appendChild(chatbotContainer);
    }

    // Render the RAGChatbot component
    const root = ReactDOM.createRoot(chatbotContainer);
    root.render(React.createElement(RAGChatbot));
  })();
}