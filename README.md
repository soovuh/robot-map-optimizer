# Robot Map Optimizer

This project focuses on developing two key functions for processing and filtering lines derived from sensor measurements and robot mapping data:  
- **`filter_duplicate_lines`**: Removes duplicate lines based on a tolerance.  
- **`filter_sub_lines`**: Filters out smaller lines that are subsets of larger lines.  

These functions ensure that the map visualization is clean and meaningful, containing only significant and unique lines.

---

## Problem Statement

Given a set of lines extracted from robot sensor data:  
1. **Duplicates**: Some lines represent the same segment but have slight variations in endpoint coordinates.  
2. **Sub-lines**: Some smaller lines are part of larger lines.  

The goal is to:  
- Remove duplicates while respecting a defined tolerance.  
- Retain only the most significant lines by eliminating sub-lines.

---

## Implementation Overview

### 1. **`filter_duplicate_lines(lines, tolerance=0.2)`**
- **Purpose**: Ensures each line in the dataset is unique by removing duplicates within the specified tolerance.
- **Input**: A list of lines, each defined as `((x1, y1), (x2, y2))`.
- **Output**: A list of unique lines.
- **Logic**:
  - Compares each line to all previously processed lines in both orientations (to account for reversed directionality).
  - Skips lines considered duplicates based on the tolerance.

### 2. **`filter_sub_lines(lines, tolerance=0.2)`**
- **Purpose**: Removes smaller lines that fall entirely within the span of larger lines.
- **Input**: A list of lines, each defined as `((x1, y1), (x2, y2))`.
- **Output**: A list of lines with sub-lines removed.
- **Logic**:
  - Normalizes lines to ensure consistent ordering of endpoints.
  - Checks if a smaller line's endpoints lie within the bounds of a larger line.

---

## Installation
1. Clone this repository:
   ```bash
   git clone https://github.com/soovuh/robot-map-optimizer.git
   ```
2. Navigate into the project directory:
   ```bash
   cd robot-map-optimizer
   ```
3. Switch to develop branch:
   ```bash
   git checkout develop
   ```
4. Install dependencies:
   ```bash
    pip install -r requirements.txt
   ```

---

## Files Overview

- **`draw_map.py`**: The main script containing the `drawMap` class and functions.  
- **`requirements.txt`**: Lists all required Python packages for the project.  
- **`SLAM/slam_map.pgm` & `SLAM/slam_map.yaml`**: Example map data for testing.  
- **`measurements.yaml`**: Sample sensor measurements used for generating map lines.  
- **`OUTPUT/`**: Folder with output images

---

