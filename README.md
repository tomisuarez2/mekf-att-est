# Multicopter Attitude Estimation using MEKF

## Overview

This repository presents a MATLAB implementation of a **Multiplicative Extended Kalman Filter (MEKF)** for multicopter attitude estimation.

The objective of this project is to demonstrate a solid understanding of:

* Nonlinear state estimation
* Sensor fusion using IMU data
* Error-state Kalman filtering
* Rigid body orientation representation using quaternions

---

## Problem Description

In low-cost inertial navigation systems, the orientation of a rigid body cannot be measured directly with sufficient accuracy.

* Gyroscopes provide high-frequency angular velocity measurements but suffer from **bias and drift**.
* Accelerometers and magnetometers provide **absolute references**, but are noisy and sensitive to disturbances.

The goal is to combine these measurements to obtain a robust and consistent estimate of the system attitude.

---

## Approach

The estimation problem is addressed using a **Multiplicative Extended Kalman Filter (MEKF)** in an **error-state formulation**.

Key features of the approach:

* Attitude represented using **unit quaternions**
* Gyroscope bias included in the state vector
* Error modeled in a **minimal representation (3D)**
* Multiplicative correction to preserve quaternion structure
* Continuous-time model with discrete-time implementation

---

## Simulation

The filter is validated in simulation using:

* Ground truth from a multicopter dynamic model
* Noisy IMU and magnetometer measurements
* Comparison against:

  * True attitude
  * TRIAD algorithm

### Results:

* Attitude estimation error remains below **1 degree**
* Drift from gyroscope integration is effectively corrected
* High-frequency noise remains (as expected)

---

## Repository Structure

```
├── src/           # MATLAB implementation
├── simulations/   # Test scenarios and scripts
└── results/       # Plots and evaluation
```
---

## Results

<div align="center">
  <img src="results/P11_cov_evol.png" width="85%" />
</div>

<div align="center">
  <img src="results/P22_cov_evol.png" width="85%" />
</div>

<div align="center">
  <img src="results/P33_cov_evol.png" width="85%" />
</div>

<div align="center">
  <img src="results/44_cov_evol.png" width="85%" />
</div>

<div align="center">
  <img src="results/55_cov_evol.png" width="85%" />
</div>

<div align="center">
  <img src="results/66_cov_evol.png" width="85%" />
</div>

<div align="center">
  <img src="results/quat_0_est.png" width="85%" />
</div>

<div align="center">
  <img src="results/quat_1_est.png" width="85%" />
</div>

<div align="center">
  <img src="results/quat_2_est.png" width="85%" />
</div>

<div align="center">
  <img src="results/quat_3_est.png" width="85%" />
</div>

<div align="center">
  <img src="results/gyro_bias_1_est.png" width="85%" />
</div>

<div align="center">
  <img src="results/gyro_bias_2_est.png" width="85%" />
</div>

<div align="center">
  <img src="results/gyro_bias_3_est.png" width="85%" />
</div>

<div align="center">
  <img src="results/attitude_estimation_error.png" width="85%" />
</div>


```bash
>>> Estimation Performance:
    - Mean attitude error: 0.5175 deg
    - Max attitude error: 3.4381 deg
    - RMS attitude error: 0.6854 deg
```
---

## References

* Crassidis & Junkins — *Optimal Estimation of Dynamic Systems*

---

## Key Takeaways

This project demonstrates:

* Practical implementation of Kalman filtering
* Sensor fusion for real-world systems

---

## Future Work

* Real sensor data validation
* Adaptive noise tuning
* Integration with control loops
* Implementation in C++ / embedded systems

---

## Contact

If you have questions or want to collaborate, feel free to reach out:
**Tomás Suárez**
Mechatronics Engineering Student
📧 [suareztomasm@gmail.com](mailto:suareztomasm@gmail.com)

