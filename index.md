# ENOS (ENvironment-adaptive On-device Software system for surveillance equipment)

## Overview
The project titled "Development of Environment-Adaptive On-Device Software Technology for Surveillance Equipment Mounted on Unmanned Vehicles" aims to develop an environment-adaptive on-device intelligence software for air threat surveillance equipment (hereafter referred to as "Environment-Adaptive On-Device Software System for Surveillance Equipment"), which is designed to be mounted on unmanned vehicles and operate in fault-disconnected environments.
<br/><br/>
The "Environment-Adaptive On-Device Software System for Surveillance Equipment" (hereafter referred to as ENOS) is an intelligent software system designed to be mounted on various surveillance equipment, including unmanned vehicle surveillance systems. ENOS adaptively responds to environmental changes and performs missions. Through group-level collaboration, ENOS enables effective management, sharing, and utilization of surveillance equipment resources, ensuring successful mission execution even in extreme environments.

## Objectives
The objectives of ENOS are as follows:
- To ensure autonomous and intelligent mission execution capabilities for surveillance equipment.
- To establish a system capable of stable operation even in extreme environments.
- To enhance mission execution efficiency through collaboration among surveillance equipment.
- To provide an integrated operational environment with existing surveillance equipment systems.

## Key Features
The core functions provided by ENOS are as follows:
- **Group Management and Communication:** Formation and management of ENOS groups, communication systems designed for extreme environments, and self-healing network capabilities.
- **Resource Management:** Real-time resource status monitoring, resource sharing and integration within the group, and legacy system resource integration.
- **Mission Execution:** Dynamic intelligence-based mission planning, real-time mission execution and monitoring, and adaptive mission reconfiguration.
- **System Integration:** Integration with legacy systems and future support for external management systems and command and control systems.

## System Architecture
Our system consists of six primary layers:
1. **Application Layer:** The topmost layer of the system, responsible for defining and managing equipment-specific information for surveillance devices equipped with ENOS and providing functions based on that information.
2. **Dynamic Intelligence Layer:** The layer responsible for autonomous decision-making and adaptive operation of the system, performing dynamic reconfiguration and optimization based on situational changes.
3. **On-Device Intelligence Software Framework Layer:** A core framework that executes the decisions of the dynamic intelligence layer and efficiently manages ENOS resources.
4. **Middleware and Library Layer:** Provides the foundation for the software framework layer, including middleware for unmanned vehicles, distributed real-time middleware, DBMS, and libraries.
5. **Operating System Layer:** Defines elements related to the OS on which ENOS operates and provides the interface between hardware and software.
6. **Network Layer:** Ensures reliable transmission and reception of data.

## Getting Started
```bash
# Clone the repository
git clone https://github.com/etri-ondev/onDevice.git

# Build the project
cd enos
mkdir build && cd build
cmake ..
make
