# Description
This repository is used to support the following work:

Microfluidic integrated DNA neural computing
Mengyao Cao, Na Li, Xiewei Xiong*, Sichen Xing, Jianing Wang, Bowen Li, Yun Zhu, Tong Zhu, Fei Wang, Siyi Hu, Hanbin Ma, Longqian Xu, Chunyu Chang, Jiajian Ji, Qunyan Yao, Li Li, Hao Pei*, Chunhai Fan*

# Droplet Path Planning for DMIC

A multi-droplet parallel path planning algorithm implemented in Qt/C++ for digital microfluidic (DMF) chips, with collision detection and avoidance.

## Overview

This project implements automatic path generation for multiple droplets moving from start positions to target positions on a 720×1280 grid-based electrode array. The algorithm selects optimal movement strategies based on droplet regions and target positions, performing collision detection at each step to maintain safe spacing between droplets.

## Core Algorithm

### Path Type Classification

Based on the droplet's starting region (upper/lower half) and relative target direction, paths are classified into 4 types:

| Type | Region | Movement Strategy |
|------|--------|-------------------|
| Type 1 | Upper half (row < 360) | Move down to channel row → move right → move down to target row → move right to destination |
| Type 2 | Upper half (row < 360) | Move down to channel row → move left → move down to target row → move right to destination |
| Type 3 | Lower half (row ≥ 360) | Move up to channel row → move right → move up to target row → move left to destination |
| Type 4 | Lower half (row ≥ 360) | Move up to channel row → move left → move up to target row → move left to destination |

### State Machine

Each droplet uses a `state` field (0→1→2→3→4) to track its current movement phase. `state=4` indicates the droplet has reached its destination.

### Collision Detection

Before each move, the algorithm checks a **2-cell safety margin** around the target position for other droplets. If occupied, the droplet stays in place and retries in the next iteration.

## File Structure

```
.
├── Sourcecode.cpp        # Core implementation (struct definitions + algorithm functions)
└── README.md
```

## Data Structures

### Drop (Droplet)

```cpp
struct Drop {
    int id;      // droplet ID
    int row;     // row coordinate
    int col;     // column coordinate
    int width;   // width (grid cells)
    int height;  // height (grid cells)
    int state;   // motion state (0~4)
    int type;    // path type (1~4)
};
```

### Path (Step)

```cpp
struct Path {
    QVector<Drop> dropInfo;  // positions of all droplets in this step
    int delay;               // step delay (ms)
};
```

## Functions

| Function | Description |
|----------|-------------|
| `CreatPath()` | Main path generation function, computes complete paths for all droplets |
| `moveReg(Drop d1, Drop d2)` | Attempts to move a droplet from d1 to d2 with collision detection, returns true on success |
| `UpdateRegMap(Drop &d1, Drop &d2)` | Updates occupancy map: clears old position, marks new position |
| `LoadChannelConfig()` | Loads channel position parameters from a config file |

## Configuration File Format

The channel config file is a `.txt` file with a single line of comma-separated values (6 parameters):

```
m_upLeft,m_upRight,m_downLeft,m_downRight,m_up,m_down
```

| Parameter | Description |
|-----------|-------------|
| `m_upLeft` | Upper-half left-channel target row |
| `m_upRight` | Upper-half right-channel target row |
| `m_downLeft` | Lower-half left-channel target row |
| `m_downRight` | Lower-half right-channel target row |
| `m_up` | Upper avoidance distance |
| `m_down` | Lower avoidance distance |

## Dependencies

- Qt 5.x or Qt 6.x (QVector, QMap, QString, QFile, etc.)
- C++11 or later

## Usage

1. Prepare a channel config file (.txt) with channel row coordinates and avoidance distances
2. Set `m_startList` (start droplet list) and `m_endList` (end droplet list)
3. Call `LoadChannelConfig()` to load channel parameters
4. Call `CreatPath()` to generate paths
5. Retrieve step-by-step droplet positions from `stepList`

## Algorithm Flow

```
Initialize → Sort droplets → Assign path types → Initialize occupancy map
    ↓
Loop (each step):
    For each unfinished droplet:
        Compute next target position based on type and state
        Call moveReg() for collision detection
        Success → update position
        Failure → stay in place, retry next iteration
        Reached destination → state = 4, remove from map
    ↓
All droplets arrived OR exceeded 5000 steps → terminate
```
