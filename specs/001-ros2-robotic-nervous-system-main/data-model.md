# Data Model: ROS 2 Robotic Nervous System Module

## Entities

### Chapter Content
- **Fields**:
  - id: string (unique identifier for the chapter)
  - title: string (chapter title)
  - content: string (Markdown formatted content)
  - wordCount: number (estimated word count)
  - learningObjectives: array of strings (what the learner should understand)
  - concepts: array of strings (key concepts covered)
  - dependencies: array of strings (prerequisites from other chapters/modules)

- **Relationships**: 
  - Belongs to Module (ROS 2 Robotic Nervous System)
  - May reference other chapters for cross-linking

- **Validation rules**:
  - title must be 5-100 characters
  - content must be between 1000-1500 words per chapter (to meet 3000-4500 total)
  - learningObjectives must have 2-5 items
  - wordCount must match actual content length (within 10%)

### Module
- **Fields**:
  - id: string (unique identifier for the module)
  - title: string (module title)
  - description: string (brief description of the module)
  - chapters: array of Chapter Content references
  - totalWordCount: number (sum of all chapter word counts)
  - targetAudience: array of strings (describing the intended learners)

- **Relationships**:
  - Contains multiple Chapter Content entities
  - May have dependencies on other modules

- **Validation rules**:
  - title must be 5-100 characters
  - totalWordCount must be between 3000-4000 words
  - must have exactly 3 chapters as specified

### Concept
- **Fields**:
  - id: string (unique identifier for the concept)
  - name: string (concept name)
  - definition: string (brief definition)
  - examples: array of strings (examples of the concept in use)
  - relatedConcepts: array of strings (other concepts this relates to)

- **Relationships**:
  - Referenced by multiple Chapter Content entities
  - Related to other Concept entities

- **Validation rules**:
  - name must be 2-50 characters
  - definition must be 10-200 characters
  - examples should have 1-3 items

### Learning Objective
- **Fields**:
  - id: string (unique identifier for the objective)
  - description: string (what the learner should be able to do/understand)
  - chapterId: string (which chapter this objective belongs to)
  - module: string (which module this objective belongs to)
  - successCriteria: string (how to measure if the objective is met)

- **Relationships**:
  - Associated with specific Chapter Content
  - Part of a Module

- **Validation rules**:
  - description must be 10-100 characters
  - successCriteria must be measurable and specific