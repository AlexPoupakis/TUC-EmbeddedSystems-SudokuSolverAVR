# TUC - Embedded Systems - Sudoku Solver AVR
Sudoku solver project for the Atmega16 AVR microcontroller developed for the undergraduate course "Embedded Systems" of the Electrical and Computer Engineering school at the Technical University of Crete.

## Overview
This project implements a performance-optimized Sudoku solver and the supporting system for the AVR architecture. It consists of a USART communication module, the Sudoku engine, and an 8-LED progress bar. The implementation exceeds the given requirements the USART communication and the progress bar, while extensively optimizing the backtracking algorithm. A detailed explanation of the project and the implementation can be found in the report.

## Key ideas
- Set-based constraint representation
- Set & one-hot digit encoding
- Fast iteration over legal digits
- Spatiotemporal locality
- Recursion elimination
- Cell-loop unrolling & elimination
- Cyclostationary register stack
- Set bitmask caching
- Lazy & static memory access
- Mask redundancy 
- Software interrupt / hook

## Results
This implementation achieved the best execution times for all (5) tested Sudoku puzzles and amongst all other teams in the course.

| Difficulty | This solver   | Other teams   |
| ---------- | ------------- | ------------- |
| Easiest    |   |   |
| Hardest    |   |   |
