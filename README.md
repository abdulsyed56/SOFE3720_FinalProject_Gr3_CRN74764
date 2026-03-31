# Introduction to A.I. Final Project
### Group 3 - CRN74764
Abdul Aziz Syed   
Areeb Khan  
Nehal Rauf  
---  

# Pathfinding Visualizer (A*, Greedy, UCS)

## Overview
A simple grid-based pathfinding visualizer built in Python.  
Lets you compare how **A\***, **Greedy Best-First**, and **Uniform Cost Search (UCS)** behave on the same map.

You can place a start node, end node, and walls, then watch each algorithm explore the grid and find a path.

---

## Features
- Interactive **2D grid**
- Set **start** and **end** nodes
- Draw / remove **walls**
- Run all algorithms with animation
- Visualize:
  - explored nodes
  - final path
- Compare results:
  - nodes expanded
  - path length
  - runtime

---

## Algorithms
- **A\*** → uses path cost + heuristic (most balanced)
- **Greedy Best-First** → uses heuristic only (fast but not always optimal)
- **UCS (Uniform Cost Search)** → uses path cost only (reliable but slower)

---

## How It Works
- The grid is treated as a graph (each cell = node)
- Movement allowed: up, down, left, right
- Each move has cost = 1
- Manhattan distance is used as the heuristic
- Algorithms use a priority queue to decide next node
- Final path is reconstructed using parent links

---  

## Project Structure

Pathfinding-Visualizer/
│
├── main.py                # Main program (UI + A*, Greedy, UCS logic)
├── DIAGRAMS.pdf           # Use case + class diagrams (system design)
├── README.md              # Project overview and instructions
│
└── (optional)
    ├── screenshots/       # Demo images for README/report
    └── report.pdf         # Final project report (if included)
---

### File Details
- **main.py**
  - Contains full implementation
  - Includes:
    - Grid + Node classes (data model)
    - A*, Greedy, UCS algorithms
    - Tkinter UI and animation

- **DIAGRAMS.pdf**
  - Use case diagram (user interactions)
  - Class diagram (system structure)

- **README.md**
  - Setup instructions
  - Project explanation

---

## How to Run
### Requirements
- Python 3.x

### Run
```bash
python main.py
