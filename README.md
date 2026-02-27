# Proj5 Group 4 — LiDAR Calibration Analysis

## 1. Project Overview
This project analyzes ROS 2 LiDAR calibration bag files recorded at known distances (0.5 m, 1.0 m, and 2.0 m) to evaluate sensor performance. The analysis computes summary statistics from LaserScan data and generates histogram plots for each test distance.

The main goals are:
- Measure **accuracy** (bias/error from the true distance)
- Measure **precision** (spread/noise of repeated measurements)
- Compare performance at multiple distances

---

## 2. Data Used
The following ROS 2 bag directories were analyzed:

- `data/lidar_calibration_/lidar_calibration_05m`
- `data/lidar_calibration_/lidar_calibration_1m`
- `data/lidar_calibration_/lidar_calibration_2m`

Topic used:
- `/scan` (`sensor_msgs/msg/LaserScan`)

The analysis focuses on measurements near the forward direction using:
- `target-angle = 0.0 rad`
- `angle-window = 0.1 rad` (±0.1 rad around the target angle)

This isolates a narrow forward-facing slice of the scan to estimate the distance directly in front of the LiDAR.

---

## 3. Environment / Dependencies
### System
- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- Python 3

### Python packages
- `numpy`
- `matplotlib`
- `pyyaml`

### Virtual environment (recommended)
This project was run inside a Python virtual environment (`.venv`) to avoid Ubuntu's system Python package restrictions (PEP 668).

---

## 4. Setup Instructions (Step-by-Step)

## Histogram Figures

### 0.5 m Histogram
![0.5 m histogram](analysis/figures/hist_05m.png)

### 1.0 m Histogram
![1.0 m histogram](analysis/figures/hist_1m.png)

### 2.0 m Histogram
![2.0 m histogram](analysis/figures/hist_2m.png)

## 3. Parameter Estimation and Results

### 3.1 Summary Table

| True Distance (m) | Mean Measured (m) | Sigma σ_hit (m) | Bias (m) | Samples (N) | Outliers (>3σ) | Outlier Rate (%) |
|---|---:|---:|---:|---:|---:|---:|
| 0.5 | 0.501861 | 0.016781 | +0.001861 | 1913 | 16 | 0.836 |
| 1.0 | 1.003373 | 0.001014 | +0.003373 | 1836 | 0 | 0.000 |
| 2.0 | 2.015235 | 0.002099 | +0.015235 | 1824 | 7 | 0.384 |

### 3.2 Parameter Estimation Analysis
#### 0.5 m 
The distribution shown in the 0.5 m histogram is a guassian distribution centered on 0.501861, with a small percentage of outliers found inbetween 0.2 and 0.5 meters.
#### 1.0 m
The distribution shown in the 1.0 m histogram is a guassian distribution centered on 1.003373, with no outliers.
#### 2.0 m
The distribution shown in the 2.0 m histogram is a guassian distribution centered on 2.015235, with a small percentage of outliers on both sides of the distribution.

All three analyzed bag files are biased greater than the expected value by a small amount, leading us to believe that there was a measurement error made when setting up our turtlebot. All three models showed the Gaussian noise predicted by Thrun's Beam Model, but only the 0.5m data showed the unexpected short range readings. None of the analyzed scan data showed max range or random readings.


### 4.1 Go to project folder
```bash
cd ~/Proj5_group4
