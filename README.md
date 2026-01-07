# Terminal 3D Renderer (C)
A lightweight, scratch-built 3D rendering engine that draws wireframe geometry directly in the terminal using ASCII. Built purely in C without external graphics libraries.

## Key Features
Custom Math Engine: Implements vector arithmetic (Dot/Cross products), normalization, and Rodrigues' rotation from scratch.

Pipeline: Handles 3D-to-2D perspective projection and rasterization.

Dynamic Camera: Features an orbiting camera that rotates around the scene origin.

Utils: Includes a custom Hash Map (djb2) for entity management.

## Quick Start
Compile and Run:
```bash
gcc main.c -o renderer -lm 
./renderer
```

## To Do
[ ] Implement vector coloring.

[ ] Fully integrate the Hash Map for scene management.

[ ] Add normal axis visualization.
