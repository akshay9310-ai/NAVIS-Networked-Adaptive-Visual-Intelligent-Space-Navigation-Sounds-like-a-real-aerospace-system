# 🛰️ NAVIS

### Networked Adaptive Visual & Intelligent Space Navigation

**NAVIS** is an AI-powered adaptive navigation and satellite coordination system designed for autonomous planetary rovers operating in GPS-denied environments such as Mars.

Instead of depending on continuous satellite coverage, NAVIS intelligently coordinates limited satellite resources while combining satellite PNT, visual navigation, IMU, and wheel odometry to keep the rover navigating reliably.

## 🚀 Key Features

* 🧠 AI-based rover trajectory prediction
* 🛰️ Adaptive satellite scheduling
* 📡 Satellite visibility simulation
* 🧭 Multi-sensor navigation & sensor fusion
* 📷 Visual navigation
* 🗺️ Terrain and hazard detection
* 🔄 Autonomous fallback during satellite outages
* ⚠️ Navigation uncertainty estimation
* 🛣️ Adaptive route planning

## 🔄 System Flow

```text
Mission & Rover State
        ↓
Trajectory Prediction
        ↓
Satellite Visibility Analysis
        ↓
AI Scheduler
        ↓
PNT / Imaging / Communication
        ↓
Sensor Fusion
        ↓
Position Estimation
        ↓
Navigation & Route Adjustment
        ↺
```

## 🌑 Example Scenario

```text
Rover approaches unknown terrain
          ↓
AI predicts future position
          ↓
Satellite becomes available
          ↓
Satellite observes upcoming terrain
          ↓
Hazard detected ⚠️
          ↓
Rover adjusts its route
          ↓
Satellite coverage is lost
          ↓
IMU + Camera + Wheel Odometry
          ↓
Rover continues autonomously
          ↓
Satellite returns
          ↓
Position is corrected
```

## 🎯 Objective

NAVIS aims to demonstrate that **intelligent satellite coordination can improve rover navigation reliability and mission performance while using limited satellite resources**.

## 📊 Evaluation

NAVIS can be evaluated by comparing:

| System | Description                              |
| ------ | ---------------------------------------- |
| **A**  | Rover-only navigation                    |
| **B**  | Rover + fixed satellite support          |
| **C**  | Rover + adaptive AI satellite scheduling |

### Metrics

* Position error
* Navigation uncertainty
* Satellite utilization
* Coverage
* Route deviation
* Mission completion rate

## 🛠️ Prototype

The NAVIS prototype is implemented as a software simulation containing:

* Simulated planetary terrain
* Rover simulator
* Satellite/orbit simulator
* Satellite visibility calculation
* Sensor simulation
* AI trajectory prediction
* AI satellite scheduling
* Terrain/hazard detection
* Sensor fusion
* Autonomous navigation

## 🌌 Vision

NAVIS explores a future where planetary rovers don't simply **wait for navigation infrastructure**—they intelligently coordinate available space assets and onboard sensors to navigate autonomously.

> **NAVIS — Adaptive Intelligence for Navigation Beyond Earth.**
