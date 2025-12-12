# Quickstart Guide: AI-Humanoid Textbook Development

## Overview
This guide provides the essential steps to set up and begin development of the Physical AI & Humanoid Robotics textbook project.

## Prerequisites

### System Requirements
- **Operating System**: Ubuntu 22.04 LTS, Windows 10+ with WSL2, or macOS 10.15+
- **Memory**: 8GB RAM minimum (16GB recommended for simulation)
- **Storage**: 20GB free space for development environment
- **Internet**: Stable connection for package downloads

### Software Dependencies
- **Node.js**: Version 18 or higher
- **npm**: Version 8 or higher (usually bundled with Node.js)
- **Python**: Version 3.8 or higher
- **Git**: Version 2.25 or higher
- **ROS 2**: Humble Hawksbill distribution
- **Docker**: Recommended for consistent environment setup

## Initial Setup

### 1. Clone the Repository
```bash
git clone <repository-url>
cd ai-humanoid-textbook
```

### 2. Install Docusaurus Dependencies
```bash
# Navigate to project root
npm install
```

### 3. Set up ROS 2 Environment
Choose one of the following options:

**Option A: Native Installation (Ubuntu)**
```bash
# Follow official ROS 2 Humble installation guide
sudo apt update
sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash
```

**Option B: Docker Setup (Recommended)**
```bash
# Build or pull the development container
docker build -t ai-humanoid-dev -f Dockerfile.dev .
# Or pull from registry
docker pull <registry>/ai-humanoid-dev:latest
```

### 4. Initialize Development Environment
```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash  # For native installation

# Or run in Docker container
docker run -it --rm -v $(pwd):/workspace ai-humanoid-dev
```

## Docusaurus Development Server

### Start Local Development
```bash
npm start
```
This command starts a local development server at `http://localhost:3000` with hot reloading.

### Build Static Site
```bash
npm run build
```
This generates the static site in the `build/` directory.

### Deploy Preview
```bash
npm run serve
```
Serves the built static site for local preview.

## Project Structure Navigation

### Key Directories
```
docs/              # Main textbook content
├── intro.md       # Introduction chapter
├── module-1-ros/  # ROS 2 fundamentals
├── module-2-sim/  # Simulation content
├── module-3-isaac/ # Isaac platform
├── module-4-vla/  # Vision-Language-Action
└── capstone/      # Capstone project

src/               # Docusaurus custom components
static/            # Static assets (images, files)
specs/             # Project specifications
└── 001-ai-humanoid-textbook/  # Current feature specs
```

## Creating New Content

### Add a New Chapter
1. Create a new Markdown file in the appropriate module directory:
```bash
# Example: Adding a chapter to Module 1 (ROS 2)
touch docs/module-1-ros/my-new-chapter.md
```

2. Follow the chapter template structure:
```markdown
---
title: My New Chapter Title
description: Brief description of the chapter content
sidebar_position: X
---

# Chapter Title

## Learning Objectives
- Objective 1
- Objective 2
- Objective 3

## Introduction
[Content here]

## Main Content
[Sections with explanations, code examples, diagrams]

## Summary
[Key takeaways]

## Exercises
1. Exercise 1
2. Exercise 2

## References
[IEEE-formatted references]
```

### Add a Code Example
1. Create the Python/C++ file in the appropriate directory:
```bash
# Example: ROS 2 Python example
mkdir -p static/code/ros2-python
touch static/code/ros2-python/my_example.py
```

2. Reference it in your chapter:
```markdown
import CodeBlock from '@site/static/code/ros2-python/my_example.py';

```python
// Include code directly in the document
# Your ROS 2 code here
```

## Essential Commands

### Content Development
```bash
# Start development server
npm start

# Build and preview
npm run build && npm run serve

# Check for broken links
npm run build
npx linkinator build/

# Format code
npx prettier --write .
```

### Git Workflow
```bash
# Create feature branch
git checkout -b feature/chapter-name

# Commit changes
git add .
git commit -m "Add: Chapter on [topic]"

# Push and create PR
git push origin feature/chapter-name
```

## Testing Content

### Validate Code Examples
1. Test ROS 2 examples in your environment:
```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Run example
python3 static/code/ros2-python/example.py
```

### Check Citations
- Ensure all citations follow IEEE format
- Verify references are from authoritative sources
- Check for plagiarism using appropriate tools

### Validate Structure
- Ensure each chapter has proper learning objectives
- Verify exercises are included
- Confirm diagrams and figures are accessible

## Deployment

### GitHub Pages
The site is automatically deployed when changes are merged to the main branch. To deploy manually:

```bash
GIT_USER=<Your GitHub username> \
  CURRENT_BRANCH=main \
  USE_SSH=true \
  npm run deploy
```

### Continuous Deployment
- Changes to `main` branch trigger automatic deployment
- Pull requests are built and previewed automatically
- Build status is visible in GitHub Actions

## Troubleshooting

### Common Issues

**Docusaurus Build Errors**:
- Ensure all dependencies are installed
- Check for syntax errors in Markdown files
- Verify all referenced files exist

**ROS 2 Environment Issues**:
- Confirm ROS 2 is properly sourced
- Check for missing ROS packages
- Verify Python environment is activated

**Simulation Environment Issues**:
- Ensure GPU drivers are properly installed (for Isaac Sim)
- Check system resource availability
- Verify Docker setup for containerized environments

### Getting Help
- Check the `/specs/001-ai-humanoid-textbook/` directory for detailed specifications
- Review the constitution at `.specify/memory/constitution.md`
- Consult the ROS 2 and Docusaurus documentation