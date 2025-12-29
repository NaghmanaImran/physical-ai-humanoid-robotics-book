const fs = require('fs');
const path = require('path');

// Function to read all MD/MDX files from docs directory
function readDocsContent(docsPath) {
  const docs = [];
  
  // Read all files in the docs directory recursively
  function readDir(currentPath) {
    const items = fs.readdirSync(currentPath);
    
    for (const item of items) {
      const fullPath = path.join(currentPath, item);
      const stat = fs.statSync(fullPath);
      
      if (stat.isDirectory()) {
        readDir(fullPath); // Recursive call for subdirectories
      } else if (path.extname(item) === '.md' || path.extname(item) === '.mdx') {
        // Read the file content
        const content = fs.readFileSync(fullPath, 'utf-8');
        
        // Extract title from the first H1 in the file or use filename
        let title = '';
        const titleMatch = content.match(/^#\s+(.*)$/m);
        if (titleMatch) {
          title = titleMatch[1].trim();
        } else {
          title = path.basename(item, path.extname(item));
        }
        
        // Create a URL path from the file path
        let relativePath = path.relative(docsPath, fullPath);
        relativePath = relativePath.replace(/\\/g, '/'); // Normalize path separators
        const urlPath = `/docs/${relativePath.replace(/\.(md|mdx)$/, '')}`;
        
        // Create document object
        docs.push({
          id: relativePath,
          title: title,
          content: content,
          url: urlPath
        });
      }
    }
  }
  
  readDir(docsPath);
  return docs;
}

// Generate the documentation index
const docsPath = path.join(__dirname, '../docs');
const docs = readDocsContent(docsPath);

// Write the index to a JSON file that can be imported by the RAGChatbot
const outputPath = path.join(__dirname, '../src/utils/docs-index.json');
fs.writeFileSync(outputPath, JSON.stringify(docs, null, 2));

console.log(`Generated documentation index with ${docs.length} documents`);