# Quickstart Guide: ROS 2 Module Development

## Prerequisites

Before starting development of the ROS 2 module, ensure you have the following installed:

- Node.js 18+ and npm/yarn
- Python 3.8+
- ROS 2 Humble Hawksbill (or Foxy Fitzroy)
- Git
- A code editor (VS Code recommended)

## Setup Docusaurus Environment

1. **Clone or create the website directory**:
   ```bash
   mkdir website
   cd website
   ```

2. **Initialize Docusaurus**:
   ```bash
   npx create-docusaurus@latest website classic --typescript
   cd website
   ```

3. **Install additional dependencies**:
   ```bash
   npm install --save-dev @docusaurus/module-type-aliases @docusaurus/tsconfig @docusaurus/preset-classic
   ```

## ROS 2 Environment Setup

1. **Install ROS 2 Humble** following the official installation guide for your platform:
   - Ubuntu: http://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
   - Windows: http://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html
   - Docker: Available for consistent development environments

2. **Source ROS 2 environment**:
   ```bash
   source /opt/ros/humble/setup.bash  # On Ubuntu
   # Or follow Windows equivalent
   ```

3. **Install Python dependencies**:
   ```bash
   pip3 install rclpy
   pip3 install rosidl_parser
   pip3 install ament_index_python
   ```

## Project Structure Setup

Create the following directory structure in your Docusaurus website:

```
website/
├── docs/
│   ├── intro.md
│   ├── chapter-1/
│   │   ├── index.md
│   │   └── concepts.md
│   ├── chapter-2/
│   │   ├── index.md
│   │   ├── nodes.md
│   │   ├── topics.md
│   │   └── services.md
│   ├── chapter-3/
│   │   ├── index.md
│   │   ├── rclpy-architecture.md
│   │   └── python-agent-control.md
│   └── chapter-4/
│       ├── index.md
│       ├── urdf-concepts.md
│       └── humanoid-models.md
├── static/
│   └── img/
└── src/
    └── components/
```

## Running the Development Server

1. **Start the Docusaurus development server**:
   ```bash
   cd website
   npm start
   ```

2. **The website will be available at**: http://localhost:3000

## Creating Content

### Adding a New Chapter Section

1. Create a new Markdown file in the appropriate chapter directory
2. Add frontmatter with metadata:
   ```markdown
   ---
   title: Your Section Title
   sidebar_position: 2
   description: Brief description of the section
   ---
   ```

### Adding Code Examples

Use Docusaurus code blocks with appropriate language tags:

```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
```

### Adding Citations

For APA-style citations, use this format in your content:
```
According to Smith et al. (2023), ROS 2 provides improved communication patterns...
```

With a reference list at the end of each chapter.

## Testing Code Examples

1. **Create a test environment** that matches the target ROS 2 version (Humble/Foxy)
2. **Validate each code example** by running it in the ROS 2 environment
3. **Document any dependencies** required for each example
4. **Test the setup instructions** to ensure reproducibility

## Building and Deployment

1. **Build the static site**:
   ```bash
   npm run build
   ```

2. **Test the build locally**:
   ```bash
   npm run serve
   ```

3. **Deploy to GitHub Pages** following Docusaurus deployment guides

## Validation Checklist

Before considering the module complete:

- [ ] All 4 chapters are written with appropriate content
- [ ] At least 3 runnable examples across nodes/topics/services
- [ ] One complete Python-to-ROS control bridge example
- [ ] One validated humanoid URDF example
- [ ] All code examples tested on ROS 2 Humble/Foxy
- [ ] Content spans 2,000-3,500 words
- [ ] All citations follow APA format
- [ ] At least 50% of sources are peer-reviewed
- [ ] Docusaurus build completes without errors
- [ ] All links and cross-references work correctly