# 🎯 Ball and Plate Controller – Stewart Platform Based

This repository contains the implementation of a control strategy for a **Ball and Plate system** mounted on a **6-DOF Stewart platform** using rotary actuators.

> 🔧 Course: CP241 – Applied Linear and Nonlinear Control Lab  
> 🧑‍🏫 Instructor: Prof. Ravi Prakash  
> 👥 Team: Soumyadipta Nath, G. Magesh, Rithwik Pradeep, Ganga Nair B  
> 📅 Semester: Jan 2025, IISc

---

## 📝 Project Summary

The objective was to control the orientation of the plate so that a ball placed on it follows a desired **trajectory** (e.g., circle, square) on the surface.

We explored and implemented the following control strategies:

- ✅ **PID Control**
- ✅ **Linear Quadratic Regulator (LQR)**
- ✅ **Sliding Mode Control (SMC)**

All controllers were implemented in **MATLAB/Simulink**. The system model was derived using **Euler–Lagrange equations**, and inverse kinematics was applied to compute platform actuator angles.

---

## 📁 Contents

- `Ball_and_plate_controller_project.pptx` – Detailed report and results
- `MATLAB/` – Simulink models and scripts for PID, LQR, and SMC controllers

---

## 📌 Highlights

- Compared controller performance for trajectory tracking
- SMC showed the best results in terms of accuracy and robustness
- Project focused on both theoretical modeling and practical control design

---

## 📎 References

See the final slide of the PPT for academic references used in this project.
