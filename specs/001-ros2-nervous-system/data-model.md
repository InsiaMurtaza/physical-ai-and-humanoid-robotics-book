# Data Model: ROS 2 Module Content Structure

## Content Entities

### Chapter
- **name**: string - Title of the chapter
- **description**: string - Brief overview of the chapter content
- **objectives**: array of strings - Learning objectives for the chapter
- **sections**: array of Section references - Content sections within the chapter
- **examples**: array of Example references - Code examples in the chapter
- **diagrams**: array of Diagram references - Visual aids in the chapter
- **validation_criteria**: array of strings - How to validate chapter completion

### Section
- **title**: string - Section title
- **content**: string - Markdown content of the section
- **type**: enum (text, code, diagram, exercise) - Type of content
- **dependencies**: array of strings - Prerequisites for understanding
- **learning_outcomes**: array of strings - What the reader should learn

### Example
- **title**: string - Descriptive name for the example
- **description**: string - What the example demonstrates
- **language**: string - Programming language used
- **code**: string - The actual code content
- **setup_instructions**: string - How to run the example
- **expected_output**: string - What the example should produce
- **validation_steps**: array of strings - How to verify the example works

### Diagram
- **title**: string - Descriptive name for the diagram
- **type**: enum (architecture, sequence, flowchart, kinematic) - Type of diagram
- **description**: string - What the diagram illustrates
- **source**: string - Source code (Mermaid, Draw.io, etc.)
- **caption**: string - Caption to appear with the diagram

### Citation
- **id**: string - Unique identifier for the citation
- **type**: enum (book, journal, website, documentation, conference) - Type of source
- **title**: string - Title of the source
- **authors**: array of strings - Authors of the source
- **year**: integer - Publication year
- **url**: string - URL if applicable
- **access_date**: string - Date when source was accessed
- **apa_citation**: string - Full APA-style citation

## Content Relationships

### Chapter contains Sections
- One-to-many relationship: Each chapter contains multiple sections
- Sections are ordered within a chapter via sequence number

### Chapter contains Examples
- One-to-many relationship: Each chapter contains multiple examples
- Examples are linked to relevant sections

### Chapter contains Diagrams
- One-to-many relationship: Each chapter contains multiple diagrams
- Diagrams are referenced by sections that use them

### Sections contain Citations
- Many-to-many relationship: Sections can reference multiple citations
- Citations can be referenced by multiple sections

## Validation Rules

### Chapter Validation
- Must have 1-4 chapters (as specified in requirements)
- Each chapter must have 3-8 sections
- Each chapter must include at least 1 example
- Each chapter must include at least 1 diagram
- Total content must be between 2,000-3,500 words across all chapters

### Section Validation
- Each section must have a clear learning objective
- Content must be coherent and focused on the section topic
- All technical claims must be supported by citations

### Example Validation
- All code examples must be tested and functional
- Examples must be compatible with ROS 2 Humble/Foxy
- Setup instructions must be complete and reproducible
- Expected outputs must match actual behavior

### Citation Validation
- All citations must follow APA format
- At least 50% of citations must be peer-reviewed sources
- All technical information must be sourced from authoritative documentation