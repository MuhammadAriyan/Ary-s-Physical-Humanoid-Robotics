# Data Model: Navigation and UI Modernization + Foundations Content

**Date**: 2025-12-07  
**Purpose**: Define content entities, UI components, and validation rules for navigation/UI modernization and comprehensive foundations content

## Core Content Entities

### Chapter Entity
```yaml
Chapter:
  id: string (unique identifier, e.g., "chapter-1-introduction")
  title: string (full chapter title)
  subtitle: string (chapter subtitle)
  part: integer (Part 1 = 1)
  chapter_number: integer (1-4)
  target_pages: integer (38-65 pages depending on chapter)
  estimated_reading_time: integer (hours)
  prerequisites: array of Chapter IDs
  learning_objectives: array of strings
  sections: array of Section IDs
  summary: text (comprehensive chapter summary)
  key_takeaways: array of strings
  exercises: array of Exercise IDs
  references: array of Reference IDs
  created_date: datetime
  last_updated: datetime
  status: enum (draft, review, published)
```

### Section Entity
```yaml
Section:
  id: string (unique identifier, e.g., "section-1-1-definitions")
  title: string (section title)
  chapter_id: string (foreign key to Chapter)
  section_number: string (e.g., "1.1", "2.3.4")
  target_pages: integer (3-25 pages)
  content_type: enum (theory, mathematics, examples, applications)
  motivation: text (historical context and motivation)
  mathematical_derivations: array of Derivation IDs
  diagrams: array of Diagram IDs
  code_examples: array of CodeExample IDs
  real_world_examples: array of RealWorldExample IDs
  exercises: array of Exercise IDs
  subsections: array of Subsection IDs
  cross_references: array of Section IDs
  created_date: datetime
  last_updated: datetime
  status: enum (draft, review, published)
```

### Mathematical Derivation Entity
```yaml
Derivation:
  id: string (unique identifier)
  title: string (derivation title)
  section_id: string (foreign key to Section)
  derivation_type: enum (proof, formula, algorithm, theorem)
  latex_content: text (LaTeX mathematical content)
  step_by_step_explanation: array of strings
  assumptions: array of strings
  applications: array of strings
  verification_method: text (how to verify the derivation)
  related_derivations: array of Derivation IDs
  difficulty_level: enum (undergraduate, graduate, research)
  created_date: datetime
  verified: boolean
  verified_by: string (expert name)
```

### Diagram Entity
```yaml
Diagram:
  id: string (unique identifier)
  title: string (diagram title)
  section_id: string (foreign key to Section)
  diagram_type: enum (kinematic, control, workflow, comparison, timeline)
  file_path: string (path to image file)
  caption: text (detailed caption)
  alt_text: string (accessibility description)
  creation_tool: enum (matlab, python, blender, manim, external)
  data_source: text (source of diagram data)
  related_diagrams: array of Diagram IDs
  resolution: string (image resolution)
  file_size: integer (bytes)
  created_date: datetime
  license: string (copyright information)
```

### Code Example Entity
```yaml
CodeExample:
  id: string (unique identifier)
  title: string (code example title)
  section_id: string (foreign key to Section)
  programming_language: enum (python, matlab, ros2, cpp)
  code_content: text (actual code)
  explanation: text (what the code does)
  input_description: text (expected inputs)
  output_description: text (expected outputs)
  dependencies: array of strings (required packages)
  execution_environment: text (required environment)
  related_concepts: array of strings (theoretical concepts)
  test_cases: array of TestCase IDs
  complexity_level: enum (beginner, intermediate, advanced)
  created_date: datetime
  tested: boolean
  test_results: text
```

### Real World Example Entity
```yaml
RealWorldExample:
  id: string (unique identifier)
  title: string (example title)
  section_id: string (foreign key to Section)
  robot_name: string (e.g., "Tesla Optimus Gen-2")
  company: string (company name)
  deployment_year: integer (year of deployment)
  application_domain: enum (manufacturing, logistics, healthcare, research)
  performance_metrics: object (key-value pairs of metrics)
  technical_specifications: object (detailed specs)
  challenges_solved: array of strings
  lessons_learned: array of strings
  references: array of Reference IDs
  images: array of Diagram IDs
  videos: array of strings (video URLs)
  created_date: datetime
  verified: boolean
```

### Exercise Entity
```yaml
Exercise:
  id: string (unique identifier)
  title: string (exercise title)
  chapter_id: string (foreign key to Chapter)
  section_id: string (foreign key to Section, optional)
  exercise_type: enum (theoretical, mathematical, programming, simulation)
  difficulty_level: enum (easy, medium, hard, challenge)
  problem_statement: text (detailed problem description)
  given_information: text (information provided to student)
  requirements: array of strings (what student needs to do)
  hints: array of strings (optional hints)
  solution: text (detailed solution)
  solution_type: enum (text, mathematical, code, simulation)
  estimated_time: integer (minutes to complete)
  prerequisites: array of strings (required knowledge)
  learning_objectives: array of strings
  created_date: datetime
  reviewed: boolean
```

### Reference Entity
```yaml
Reference:
  id: string (unique identifier)
  title: string (reference title)
  authors: array of strings
  publication_year: integer
  publication_type: enum (journal, conference, book, thesis, website, video)
  venue: string (journal name, conference name, publisher)
  doi: string (digital object identifier)
  url: string (URL if available)
  pages: string (page numbers)
  volume: string (volume number)
  issue: string (issue number)
  keywords: array of strings
  abstract: text (brief summary)
  relevance_score: integer (1-10, relevance to content)
  access_date: datetime (when accessed)
  created_date: datetime
```

## Content Relationships

### Hierarchical Structure
```
Part 1
├── Chapter 1: Introduction (38-45 pages)
│   ├── Section 1.1: Definitions & Taxonomy (8-10 pages)
│   ├── Section 1.2: Historical Timeline (12-15 pages)
│   ├── Section 1.3: Why Humanoids Matter (10-12 pages)
│   └── Section 1.4: Grand Challenges (8-10 pages)
├── Chapter 2: Mathematical Foundations (48-55 pages)
│   ├── Section 2.1: Linear Algebra Review (6-8 pages)
│   ├── Section 2.2: Rigid-Body Geometry (10-12 pages)
│   ├── Section 2.3: Rotation Representations (15-18 pages)
│   ├── Section 2.4: Lie Groups SE(3) & so(3) (12-15 pages)
│   └── Section 2.5: Probability Primer (8-10 pages)
├── Chapter 3: Kinematics and Dynamics (55-65 pages)
│   ├── Section 3.1: Forward Kinematics (12-15 pages)
│   ├── Section 3.2: Jacobians (12-15 pages)
│   ├── Section 3.3: Inverse Kinematics (10-12 pages)
│   ├── Section 3.4: Singularity Analysis (8-10 pages)
│   ├── Section 3.5: Lagrangian Dynamics (15-20 pages)
│   └── Section 3.6: Newton-Euler Algorithm (10-12 pages)
└── Chapter 4: Simulation and Digital Twins (35-42 pages)
    ├── Section 4.1: Modern Simulators Overview (10-12 pages)
    ├── Section 4.2: Articulated-Body Dynamics (10-12 pages)
    ├── Section 4.3: Contact Modeling (8-10 pages)
    ├── Section 4.4: Sim-to-Real Transfer (12-15 pages)
    └── Section 4.5: Digital-Twin Pipelines (8-10 pages)
```

## Validation Rules

### Content Quality Constraints
```yaml
ValidationRules:
  chapter:
    min_pages: 35
    max_pages: 65
    min_sections: 3
    max_sections: 6
    min_exercises: 15
    min_references: 20
    
  section:
    min_pages: 3
    max_pages: 25
    min_diagrams: 2
    max_diagrams: 5
    min_code_examples: 0
    max_code_examples: 3
    min_real_world_examples: 1
    min_exercises: 2
    
  derivation:
    required_fields: [title, latex_content, step_by_step_explanation]
    latex_validation: true
    expert_verification_required: true
    
  diagram:
    min_resolution: "300dpi"
    max_file_size: 5242880  # 5MB
    required_alt_text: true
    required_caption: true
    
  code_example:
    syntax_validation: true
    execution_test_required: true
    dependency_specification_required: true
    
  exercise:
    solution_required: true
    difficulty_level_specified: true
    estimated_time_specified: true
    
  reference:
    min_relevance_score: 5
    publication_year_range: [2019, 2025]
    required_fields: [title, authors, publication_year, publication_type]
```

### Constitutional Compliance Rules
```yaml
ConstitutionalCompliance:
  academic_rigor:
    university_level_required: true
    mathematical_foundations_required: true
    current_research_required: true
    citation_range: [15, 40]  # per section
    
  physical_embodiment_focus:
    hardware_emphasis_required: true
    real_world_applications_required: true
    humanoid_specific_challenges_required: true
    
  visual_practical_learning:
    diagrams_per_section_range: [2, 5]
    real_world_case_studies_required: true
    code_snippets_required: true
    practical_exercises_required: true
    
  state_of_the_art_integration:
    current_advancements_required: true
    industrial_deployments_required: true
    recent_course_materials_required: true
    
  comprehensive_coverage:
    interdisciplinary_connections_required: true
    appropriate_depth_required: true
    expansive_subsections_required: true
    
  ethical_inclusive_design:
    ethical_implications_required: true
    accessibility_considerations_required: true
    diverse_applications_required: true
```

## Data Integrity Constraints

### Uniqueness Constraints
- Chapter IDs must be unique across all parts
- Section IDs must be unique within each chapter
- Diagram IDs must be unique across all content
- Code example IDs must be unique across all content
- Exercise IDs must be unique within each chapter

### Referential Integrity
- All section.chapter_id references must exist
- All diagram.section_id references must exist
- All code_example.section_id references must exist
- All exercise.chapter_id and exercise.section_id references must exist
- All derivation.section_id references must exist

### Data Consistency
- Section numbers must follow hierarchical pattern
- Page counts must sum to chapter totals within ±10%
- All mathematical derivations must be verified before publication
- All code examples must be tested before publication
- All real-world examples must be from 2019-2025

## Performance Considerations

### Content Delivery
- Diagram files should be optimized for web delivery
- Code examples should be syntax-highlighted and properly formatted
- Mathematical content should render efficiently using MathJax
- Page load times should be under 3 seconds for typical content

### Search and Discovery
- All content should be full-text searchable
- Mathematical equations should be searchable when possible
- Code examples should be searchable by language and concept
- Real-world examples should be searchable by robot and company

### Accessibility
- All diagrams must have descriptive alt text
- Mathematical content should have screen reader support
- Code examples should be accessible via keyboard
- Content should support multiple learning styles

## Migration and Versioning

### Content Versioning
- Each content entity should have version history
- Major changes should create new versions
- Minor edits should update existing versions
- All changes should be tracked with attribution

### Backup and Recovery
- Content should be regularly backed up
- Version history should be preserved
- Recovery procedures should be documented
- Content should be exportable in multiple formats

## Navigation and UI Components

### Content Index Entity
```yaml
ContentIndex:
  id: string (unique identifier)
  generated_at: datetime
  total_chapters: integer
  total_sections: integer
  last_updated: datetime
  content_tree: object (hierarchical structure)
  search_index: object (full-text search data)
  categories: array of strings
  tags: array of strings
```

### UI Component Entity
```yaml
UIComponent:
  id: string (unique identifier)
  component_type: enum (WelcomePage, ContentGrid, SearchBar, Navigation)
  props_schema: object (component properties)
  styling: object (CSS-in-JS styling)
  animations: object (Framer Motion animations)
  responsive_breakpoints: array of objects
  accessibility_features: array of strings
  performance_metrics: object (load time, interaction metrics)
```

### Theme Entity
```yaml
Theme:
  id: string (unique identifier)
  name: string (theme name)
  color_palette: object (primary, secondary, accent colors)
  typography: object (font families, sizes, weights)
  spacing: object (margin, padding scales)
  animations: object (timing functions, durations)
  breakpoints: object (responsive breakpoints)
  custom_properties: object (CSS custom properties)
```

### Navigation State Entity
```yaml
NavigationState:
  current_path: string
  search_query: string
  selected_category: string
  expanded_sections: array of strings
  recent_pages: array of strings
  bookmarks: array of strings
  user_preferences: object (theme, layout preferences)
```

## UI-Specific Validation Rules

### Performance Constraints
```yaml
UIPerformance:
  max_initial_load_time: 3000  # milliseconds
  max_navigation_transition: 1000  # milliseconds
  min_lighthouse_performance: 90
  max_bundle_size: 1048576  # 1MB
  min_accessibility_score: 95
```

### Design System Constraints
```yaml
DesignSystem:
  color_contrast_ratio: 4.5  # WCAG AA compliance
  font_scale_ratio: 1.25  # modular scale
  spacing_unit: 8  # base spacing unit in pixels
  border_radius_scale: [4, 8, 12, 16]  # consistent border radius
  animation_duration_range: [150, 300, 500]  # milliseconds
```

### Responsive Design Requirements
```yaml
ResponsiveDesign:
  mobile_breakpoint: 768  # pixels
  tablet_breakpoint: 1024  # pixels
  desktop_breakpoint: 1440  # pixels
  touch_target_size: 44  # minimum pixels
  readable_line_length: [45, 75]  # characters
```

## Constitutional Compliance for UI

### Homepage Content Integration
```yaml
HomepageIntegration:
  auto_content_discovery: true
  dynamic_navigation_generation: true
  prominent_section_display: true
  immediate_content_discoverability: true
  manual_update_intervention: false
```

### Modern UI Design Requirements
```yaml
ModernUI:
  glassmorphism_enabled: true
  neumorphism_enabled: true
  gradient_overlays: true
  smooth_animations: true
  micro_interactions: true
  responsive_layouts: true
  contemporary_typography: true
  sophisticated_color_palettes: true
```

### Custom Branding Requirements
```yaml
CustomBranding:
  default_docusaurus_removal: true
  custom_theming_implementation: true
  professional_typography: true
  academic_visual_identity: true
  unique_visual_distinction: true
```

This comprehensive data model provides foundation for creating modern, beautiful UI with dynamic navigation while maintaining comprehensive educational content that meets all constitutional requirements including homepage integration, modern design aesthetics, and custom branding.