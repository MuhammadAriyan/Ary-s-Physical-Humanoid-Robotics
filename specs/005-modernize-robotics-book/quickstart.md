# Quickstart Guide: Contributing to the Robotics Book

This guide provides instructions for contributors on how to add new content to the robotics book.

## 1. Prerequisites

-   Familiarity with Git and GitHub.
-   A Markdown editor of your choice.
-   Node.js and npm installed.

## 2. Getting Started

1.  **Fork the repository**: Create a fork of the main repository on GitHub.
2.  **Clone your fork**: Clone your forked repository to your local machine.
3.  **Install dependencies**: Run `npm install` in the root directory to install the necessary dependencies.

## 3. Adding New Content

1.  **Create a new branch**: Create a new branch for your changes.
2.  **Create a new Markdown file**: Create a new `.md` file in the appropriate directory under `docs`.
3.  **Follow the chapter structure**: Ensure your new chapter follows the structure defined in the constitution.
4.  **Update the sidebar**: Add a link to your new chapter in the `sidebars.ts` file.
5.  **Add a link to the home page**: Add a link to your new chapter on the home page (`src/pages/index.tsx`).

## 4. Submitting Your Changes

1.  **Commit your changes**: Commit your changes with a descriptive commit message.
2.  **Push to your fork**: Push your changes to your forked repository.
3.  **Create a pull request**: Create a pull request from your fork to the main repository.

## 5. Content Style Guide

-   **Language**: Write in clear, concise, and professional English.
-   **Tone**: Maintain an educational and authoritative tone.
-   **Formatting**: Use Markdown for formatting.
-   **Code**: All code examples must be well-commented and runnable.
-   **Diagrams**: All diagrams must be clear, professional, and consistent in style.
