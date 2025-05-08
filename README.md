# Lane-Change Decision-Making for Autonomous Vehicles

This project presents a comparative study of rule-based, machine learning, and hybrid models for autonomous vehicle lane-change decision-making (LCDM), evaluated across realistic traffic scenarios using the SUMO simulation environment. The focus is on balancing **interpretability**, **adaptability**, and **performance**—three critical factors in safety-critical autonomous systems.

---

## Project Overview

Autonomous vehicles must safely and efficiently perform lane changes in dynamic environments. This project:

- Reimplements and enhances well-known rule-based models (SL2015, Liu et al.).
- Develops and evaluates a Support Vector Machine (SVM)-based ML model.
- Introduces an **enhanced hybrid model** that adapts rule-based decisions with dynamic traffic-aware parameters.
- Compares all models using safety, fuel efficiency, and passenger comfort as evaluation metrics.

---

## Key Features

- **SUMO + TraCI Integration**: Simulates multi-lane highways with low, medium, and high traffic.
- **Modeling Approaches**:
  - SL2015: Strategic rule-based with safety constraints.
  - Liu et al.: Original and enhanced rule-based models with dynamic coefficients.
  - SVM: Data-driven decision-making based on trajectory features.
- **Evaluation Metrics**:
  - **Safety**: Collision checks and headway distances.
  - **Efficiency**: Fuel consumption (km/L).
  - **Comfort**: Longitudinal jerk analysis.

---

## Technologies

- **Python**, **scikit-learn**, **SUMO**, **TraCI API**
- **Pandas**, **NumPy**, **Matplotlib**
- SVM classification with RBF kernel
- XML data parsing for metric extraction

---

## Results Summary

| Model        | Safety | Efficiency (km/L) | Comfort |
| ------------ | ------ | ----------------- | ------- |
| SL2015       | ★★★☆☆  | ★★★★★             | ★★★★★   |
| Liu (Base)   | ★★★☆☆  | ★★★★☆             | ★★☆☆☆   |
| Liu Enhanced | ★★★★☆  | ★★☆☆☆             | ★★★★☆   |
| SVM (ML)     | ★★☆☆☆  | ★★☆☆☆             | ★★★☆☆   |

---

## Academic Context
This project was developed as part of Autonomous and cooperative vehicular systems course at Chalmers University of Technology, with the goal of exploring interpretable AI approaches for autonomous vehicle decision-making.

---

## Demo
Watch the demo video of the simulation under heavy traffic:
Scenario C Demo Video

---

## For detailed methodology and evaluation:
A Comparative Study of Rule-Based and Machine Learning Models for Lane-Change Decision-Making in Autonomous Vehicles

Read the full report](LaneChange_Paper.pdf)

