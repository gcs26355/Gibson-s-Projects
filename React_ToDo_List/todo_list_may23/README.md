# To-Do List

A simple to-do list app built with React and TypeScript.

## Features

- Add tasks by typing and pressing **Enter** or clicking **Submit**
- Check off tasks with an animated fade-out on completion
- Edit existing tasks inline
- Tasks persist across page refreshes via **localStorage**

## Project Structure

```
src/
├── components/
│   └── ToDos.tsx          # Main list component
└── paths/
    ├── createToDo.ts      # Adds a new task
    ├── deleteToDo.ts      # Removes a task by ID
    └── updateToDo.tsx     # Inline edit UI and logic
```

## Getting Started

```bash
npm install
npm run dev
```

## Built With

- [React](https://react.dev/)
- [TypeScript](https://www.typescriptlang.org/)
- [Vite](https://vitejs.dev/)