# Quickstart Guide: Setting up the ROS 2 Robotic Nervous System Module

## Prerequisites

- Node.js (v16 or higher)
- npm or yarn package manager
- Git for version control
- A GitHub account for deployment

## Setup Instructions

### 1. Clone the Repository
```bash
git clone [repository-url]
cd [repository-name]
```

### 2. Install Dependencies
```bash
npm install
# or
yarn install
```

### 3. Start the Development Server
```bash
npm run start
# or
yarn start
```

This command starts a local development server and opens up a browser window with the documentation. Most changes are reflected live without having to restart the server.

### 4. Navigate to the ROS 2 Module
Once the development server is running, you can access the ROS 2 Robotic Nervous System module at:
```
http://localhost:3000/docs/modules/ros2-nervous-system
```

### 5. Add or Modify Content
The three chapters of Module 1 are located in the `docs/modules/ros2-nervous-system/` directory:
- `chapter-1-intro.md` - Introduction to ROS 2 and the Robotic Nervous System
- `chapter-2-communication.md` - ROS 2 Communication Model for Humanoid Control
- `chapter-3-structure.md` - Robot Structure and Control Interfaces

### 6. Build for Production
```bash
npm run build
# or
yarn build
```

This command generates static content into the `build` directory and can be served using any static content hosting service.

### 7. Deployment
The site is configured to deploy to GitHub Pages. After building, you can deploy with:
```bash
npm run deploy
# or
yarn deploy
```

## Customization

### Sidebar Navigation
The sidebar navigation is configured in `sidebar.js`. You can modify the structure to add, remove, or reorganize documentation sections.

### Site Configuration
Site-wide settings like title, description, and theme are configured in `docusaurus.config.js`.

## Troubleshooting

### Common Issues
1. **Port already in use**: If you get an error about the port being in use, try running `npm run start -- --port 3001` to use a different port.
2. **Dependency conflicts**: If you encounter dependency issues, try deleting the `node_modules` folder and running `npm install` again.
3. **Build failures**: Check that all Markdown files have proper frontmatter and syntax.

### Getting Help
- For Docusaurus-specific questions, check the [official Docusaurus documentation](https://docusaurus.io/)
- For project-specific questions, refer to the feature specification in `specs/001-ros2-robotic-nervous-system/spec.md`