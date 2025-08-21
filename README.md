# Multithreaded Autonomous Platform Controller

C++ control system for simultaneous operation of multiple autonomous vehicles in ROS2/Gazebo simulation.

## Key Achievements

- **Parallel Control**: Independent threads managing Ackerman vehicle and quadcopter simultaneously
- **Thread-Safe Design**: Mutex locks and atomic operations ensuring data integrity
- **Real-time Performance**: 100Hz control loops with <10ms response time
- **Dynamic Progress Tracking**: Live mission completion percentage based on distance metrics

## Technical Implementation

### Architecture
- **Mission Coordinator**: Orchestrates multiple platforms with thread management
- **Ackerman Controller**: Ground vehicle with realistic steering physics (2.91 m/s max speed)
- **Quadcopter Controller**: 3D aerial navigation with altitude control (2m operating height)
- **Base Controller**: Abstract class providing shared functionality (odometry, status, motion tracking)

### Core Algorithms

**Thread Synchronisation**
```cpp
std::mutex mutex_;                    // Protects shared resources
std::atomic<PlatformStatus> status_;  // Lock-free status updates
std::thread::join()                   // Clean mission completion
```
**Motion Control**

- **Ackerman**: Proportional steering with acceleration/deceleration zones
- **Quadcopter**: Adaptive speed control with drift compensation
- **Both**: Tolerance-based waypoint achievement (0.5m precision)

## Technical Implementation

**Stack**: C++17, std::thread, ROS2 Humble, Gazebo
**Patterns**: Template Method, Strategy, Observer
**Build**: CMake, GCC

## System Design

- Non-blocking execution with immediate goal processing
- Distance-based progress calculation for accurate mission tracking
- Polymorphic controller design for platform extensibility
- RAII principles for resource management

---
*Individual project demonstrating concurrent programming and real-time control system design.*
