<div align="center">

# 💥 ROS Fault Injector
### **Chaos Engineering & Resilience Verification Middleware for Autonomous ROS 2 Systems**
*Stochastic Middleware Interposition · Poisson Packet Loss · 6-DOF IMU Perturbation · $S^3$ Lie Group Normalization*

[![CI Build & Test](https://img.shields.io/badge/CI-Passing%20(3%2F3%20Tests)-00d4ff?style=for-the-badge&logo=githubactions&logoColor=white)](https://github.com/EngineerAbdullahBinZafar/ros-fault-injector)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Iron%20%7C%20Jazzy-22314E?style=for-the-badge&logo=ros&logoColor=white)](https://docs.ros.org)
[![Reliability](https://img.shields.io/badge/Reliability-Chaos%20Engineering-f59e0b?style=for-the-badge&logo=target&logoColor=white)](https://github.com/EngineerAbdullahBinZafar/ros-fault-injector)
[![License](https://img.shields.io/badge/License-MIT-green.svg?style=for-the-badge)](LICENSE)

---

```
"If an autonomous robot has never been validated under controlled chaos, it has never truly been verified."
```

```bash
# ⚡ 1-Command Live Terminal Demonstration (Zero Dependencies)
python demo.py
```

[Executive Overview](#-executive-overview) • [Mathematical Formulations](#-mathematical-formulations--reliability-models) • [Architecture](#-chaos-engineering-architecture) • [CLI & Parameters](#%EF%B8%8F-cli-usage--runtime-parameters) • [Scenarios](#-experimental-fault-scenarios) • [Verification](#-verification--empirical-test-suite)

</div>

<br/>

## 🎯 Executive Overview

**ROS Fault Injector** is a deterministic and stochastic chaos engineering framework engineered to battle-test ROS 2 navigation, state estimation, and control pipelines before physical field deployment.

Distributed robotic architectures frequently suffer from transient physical anomalies: vibration-induced sensor white noise, WiFi/DDS packet dropouts, transceiver buffer serialization delays, and LiDAR atmospheric backscatter. Without systematic fault interposition, downstream estimators (EKFs, particle filters, Nav2 costmaps) silently drift, enter non-convergent covariance loops, or trigger unhandled safety fault escalations.

`ros-fault-injector` inserts a non-intrusive, inline interposition lifecycle node between publishers and subscribers that injects mathematically rigorous fault distributions:
- **Poisson Packet Drops:** Models independent Bernoulli packet loss across stochastic lossy channels.
- **Microsecond Latency Buffer:** Thread-safe non-blocking delay queue operating at 100 Hz.
- **Multi-Domain Sensor Perturbations:** Additive White Gaussian Noise (AWGN) for velocities and accelerations, bounded impulse salt-and-pepper noise for LiDAR range readings, and Lie group geodesic perturbations with **$S^3$ unit quaternion normalization** to preserve attitude manifold constraints.
- **Dynamic Parameter Reconfiguration:** Mutates injection rates, standard deviations, and latency targets on-the-fly via ROS 2 runtime parameter callbacks.

> ⚠️ **Critical Entrypoint Clarification:** In accordance with `setup.py`, the official console executable entrypoint is `injector_node` (i.e. `ros2 run ros_fault_injector injector_node`), not the legacy deprecated reference `fault_node`.

---

## 📐 Mathematical Formulations & Reliability Models

### 1. Poisson Packet Drop Distribution & Channel Loss

Message transmission across distributed DDS/WiFi channels with nominal publication frequency $f_s$ ($\text{Hz}$) and per-packet loss probability $p_{\text{drop}} \in [0, 1]$ is modeled as an independent Bernoulli trial per message. Over macroscopic observation intervals $\Delta t$, packet loss converges to a homogeneous Poisson process with drop intensity:

$$\lambda_{\text{drop}} = f_s \cdot p_{\text{drop}}$$

The probability of dropping exactly $k$ messages during duration $\Delta t$ is given by:

$$P(K = k; \Delta t) = \frac{(\lambda_{\text{drop}} \Delta t)^k e^{-\lambda_{\text{drop}} \Delta t}}{k!}, \quad k \in \{0, 1, 2, \dots\}$$

The continuous inter-arrival duration $T_{\text{drop}}$ between consecutive packet drop events follows an exponential distribution:

$$f_T(t) = \lambda_{\text{drop}} e^{-\lambda_{\text{drop}} t}, \quad t \ge 0$$

with expectation and variance:

$$\mathbb{E}[T_{\text{drop}}] = \frac{1}{\lambda_{\text{drop}}} = \frac{1}{f_s \cdot p_{\text{drop}}}, \quad \operatorname{Var}(T_{\text{drop}}) = \frac{1}{\lambda_{\text{drop}}^2} = \frac{1}{(f_s \cdot p_{\text{drop}})^2}$$

### 2. 6-DOF IMU Sensor Noise Covariance & Random Walk

Real-world inertial measurement units (IMUs) subject to high-frequency mechanical vibration and thermal drift are modeled by continuous-time stochastic differential equations:

$$\tilde{\mathbf{a}}(t) = \mathbf{a}_{\text{true}}(t) + \mathbf{b}_a(t) + \boldsymbol{\eta}_a(t)$$

$$\tilde{\boldsymbol{\omega}}(t) = \boldsymbol{\omega}_{\text{true}}(t) + \mathbf{b}_\omega(t) + \boldsymbol{\eta}_\omega(t)$$

where $\boldsymbol{\eta}_a(t)$ and $\boldsymbol{\eta}_\omega(t)$ represent zero-mean Additive White Gaussian Noise (AWGN):

$$\boldsymbol{\eta}_a(t) \sim \mathcal{N}\left(\mathbf{0}, \sigma_a^2 \mathbf{I}_{3 \times 3}\right), \quad \boldsymbol{\eta}_\omega(t) \sim \mathcal{N}\left(\mathbf{0}, \sigma_\omega^2 \mathbf{I}_{3 \times 3}\right)$$

The continuous 6-DOF IMU noise covariance matrix is block-diagonal:

$$\boldsymbol{\Sigma}_{\text{IMU}} = \begin{bmatrix} \sigma_a^2 \mathbf{I}_{3 \times 3} & \mathbf{0}_{3 \times 3} \\ \mathbf{0}_{3 \times 3} & \sigma_\omega^2 \mathbf{I}_{3 \times 3} \end{bmatrix} \in \mathbb{R}^{6 \times 6}$$

Under discrete numerical sampling at frequency $f_s = \frac{1}{\Delta t}$, discrete noise standard deviation scales with the sampling rate:

$$\sigma_{a,\text{disc}} = \frac{\sigma_a}{\sqrt{\Delta t}} = \sigma_a \sqrt{f_s}, \quad \sigma_{\omega,\text{disc}} = \frac{\sigma_\omega}{\sqrt{\Delta t}} = \sigma_\omega \sqrt{f_s}$$

The accelerometer and gyroscope bias vectors $\mathbf{b}_a(t)$ and $\mathbf{b}_\omega(t)$ follow a Wiener random walk (Brownian motion):

$$\dot{\mathbf{b}}_a(t) = \boldsymbol{\eta}_{ba}(t), \quad \boldsymbol{\eta}_{ba}(t) \sim \mathcal{N}\left(\mathbf{0}, \sigma_{ba}^2 \mathbf{I}_{3 \times 3}\right)$$

$$\mathbf{b}_a[k] = \mathbf{b}_a[k-1] + \boldsymbol{\eta}_{ba}[k] \sqrt{\Delta t}$$

### 3. Quaternion Lie Algebra Perturbations & $S^3$ Manifold Normalization

Robotic 3D orientation is parameterized by unit quaternions living on the 3-sphere Riemannian manifold $S^3 \subset \mathbb{H}$:

$$\mathbf{q} = \begin{bmatrix} q_w \\ \mathbf{q}_v \end{bmatrix} = \begin{bmatrix} q_w \\ q_x \\ q_y \\ q_z \end{bmatrix} \in S^3, \quad \|\mathbf{q}\|_2 = \sqrt{q_w^2 + q_x^2 + q_y^2 + q_z^2} = 1$$

To inject kinematically consistent rotational jitter without introducing non-physical attitude shear, perturbations are generated in the tangent space Lie algebra $\mathfrak{so}(3) \cong \mathbb{R}^3$:

$$\delta \boldsymbol{\theta} \sim \mathcal{N}\left(\mathbf{0}, \boldsymbol{\Sigma}_\theta\right) \in \mathbb{R}^3$$

The exponential map $\exp: \mathfrak{so}(3) \to S^3$ lifts the Lie algebra vector to a unit quaternion:

$$\delta \mathbf{q} = \exp\left( \frac{1}{2} \delta \boldsymbol{\theta} \right) = \begin{bmatrix} \cos\left(\frac{\|\delta \boldsymbol{\theta}\|}{2}\right) \\ \frac{\delta \boldsymbol{\theta}}{\|\delta \boldsymbol{\theta}\|} \sin\left(\frac{\|\delta \boldsymbol{\theta}\|}{2}\right) \end{bmatrix}$$

Applying the perturbation via Hamilton quaternion multiplication:

$$\tilde{\mathbf{q}} = \mathbf{q} \otimes \delta \mathbf{q} = \begin{bmatrix} q_w \delta q_w - \mathbf{q}_v \cdot \delta \mathbf{q}_v \\ q_w \delta \mathbf{q}_v + \delta q_w \mathbf{q}_v + \mathbf{q}_v \times \delta \mathbf{q}_v \end{bmatrix}$$

To prevent numerical floating-point divergence over prolonged simulation runs, the state is projected back onto the $S^3$ manifold:

$$\mathbf{q}_{\text{norm}} = \frac{\tilde{\mathbf{q}}}{\|\tilde{\mathbf{q}}\|_2} = \frac{\tilde{\mathbf{q}}}{\sqrt{\tilde{q}_w^2 + \tilde{q}_x^2 + \tilde{q}_y^2 + \tilde{q}_z^2}}$$

The corresponding rotation matrix $R(\mathbf{q}) \in \mathrm{SO}(3)$ preserves orthogonality:

$$R(\mathbf{q}) = \mathbf{I}_{3 \times 3} + 2 q_w [\mathbf{q}_v]_\times + 2 [\mathbf{q}_v]_\times^2, \quad R(\mathbf{q}) R(\mathbf{q})^T = \mathbf{I}_{3 \times 3}, \quad \det(R(\mathbf{q})) = +1$$

### 4. Continuous Reliability, Hazard Rate & Availability Formulations

For robotic hardware modules subjected to chaos stress-testing, time-to-failure random variable $T \ge 0$ has cumulative probability distribution $F(t) = P(T \le t)$ and probability density $f(t) = \frac{dF(t)}{dt}$.

The System Reliability (Survival) function is:

$$R(t) = 1 - F(t) = P(T > t) = \int_t^\infty f(\tau) d\tau$$

The instantaneous hazard rate (failure intensity) $\lambda(t)$ is defined as:

$$\lambda(t) = \lim_{\Delta t \to 0} \frac{P(t < T \le t + \Delta t \mid T > t)}{\Delta t} = \frac{f(t)}{R(t)} = -\frac{d}{dt} \ln R(t)$$

Integrating both sides yields the general reliability formulation:

$$R(t) = \exp\left( -\int_0^t \lambda(\tau) d\tau \right)$$

Under constant hazard rate conditions $\lambda(t) = \lambda_0$ (exponential distribution, representing useful operating life):

$$R(t) = e^{-\lambda_0 t}$$

The Mean Time Between Failures (MTBF) is:

$$\text{MTBF} = \mathbb{E}[T] = \int_0^\infty R(t) dt = \int_0^\infty e^{-\lambda_0 t} dt = \frac{1}{\lambda_0}$$

In a repairable robotics system with Mean Time to Recovery (MTTR) and recovery rate $\mu = \frac{1}{\text{MTTR}}$, operational asymptotic availability $A$ is:

$$A = \frac{\text{MTBF}}{\text{MTBF} + \text{MTTR}} = \frac{\mu}{\lambda_0 + \mu}$$

---

## 📐 Chaos Engineering Architecture

```
                                      +---------------------------------------------+
                                      |            ROS Fault Injector               |
                                      |              (injector_node)                |
                                      +---------------------------------------------+
                                                             |
   +-----------------------+      Subscribe                  v                   Publish       +-----------------------+
   |      Real Topic       | --------------------> [ Deserialization ] --------------------->  |     Degraded Topic    |
   | (e.g. /imu/data,      |                                 |                                 | (e.g. /imu/faulty,    |
   |  /cmd_vel, /scan)     |                                 |                                 |  /scan/noisy)         |
   +-----------------------+                                 |                                 +-----------------------+
                                           +-----------------+-----------------+                           |
                                           |                 |                 |                           v
                                           v                 v                 v               +-----------------------+
                                    [ Delay Queue ]   [ Bernoulli Drop ]   [ Noise Engine ]    | Downstream Estimator  |
                                    (100 Hz Timer)    (p_drop trial)       (AWGN / S^3 Norm)   | (EKF, SLAM, Nav2)     |
                                           |                 |                 |               +-----------------------+
                                           +-----------------+-----------------+
```

---

## ⚙️ CLI Usage & Runtime Parameters

### Launching the Fault Injector Node

```bash
# General invocation syntax
ros2 run ros_fault_injector injector_node \
    --ros-args \
    -p input_topic:=<INPUT_TOPIC> \
    -p output_topic:=<OUTPUT_TOPIC> \
    -p msg_type:=<Pose|Twist|LaserScan|Imu|String> \
    -p fault_type:=<delay|drop|noise|corrupt> \
    -p delay_ms:=<MILLISECONDS> \
    -p drop_probability:=<PROBABILITY_0_TO_1> \
    -p noise_std_dev:=<STANDARD_DEVIATION> \
    -p salt_pepper_prob:=<PROBABILITY_0_TO_1>
```

### Parameter Reference

| Parameter Name | Data Type | Default Value | Valid Range | Operational Description |
|:---|:---:|:---:|:---:|:---|
| `input_topic` | `string` | `'/input_topic'` | Valid topic | Input ROS 2 topic to interpose and intercept |
| `output_topic` | `string` | `'/output_topic'` | Valid topic | Output topic where perturbed messages are published |
| `msg_type` | `string` | `'Pose'` | `'Pose'`, `'Twist'`, `'LaserScan'`, `'Imu'`, `'String'` | Data type descriptor for dynamic subscriber binding |
| `fault_type` | `string` | `'delay'` | `'delay'`, `'drop'`, `'noise'`, `'corrupt'` | Primary chaos injection strategy |
| `delay_ms` | `int` | `100` | $\ge 0$ | Milliseconds of artificial latency queued before dispatch |
| `drop_probability`| `float` | `0.1` | $[0.0, 1.0]$ | Probability of dropping incoming messages (Bernoulli trial) |
| `noise_std_dev` | `float` | `0.05` | $\ge 0.0$ | Standard deviation $\sigma$ for additive zero-mean Gaussian noise |
| `salt_pepper_prob`| `float` | `0.02` | $[0.0, 1.0]$ | Probability of injecting $0.0$ or `range_max` spikes in LiDAR scan |

---

## 🎭 Experimental Fault Scenarios

### Scenario 1: Intermittent WiFi Loss on Velocity Commands (`/cmd_vel`)
Simulate high packet loss over congested wireless telemetry during teleoperation:
```bash
ros2 run ros_fault_injector injector_node \
    --ros-args \
    -p input_topic:=/cmd_vel \
    -p output_topic:=/cmd_vel_degraded \
    -p msg_type:=Twist \
    -p fault_type:=drop \
    -p drop_probability:=0.35
```

### Scenario 2: Severe 6-DOF IMU Vibration Noise During Off-Road Traversal
Inject additive Gaussian noise into linear accelerations and angular rates to test EKF stability:
```bash
ros2 run ros_fault_injector injector_node \
    --ros-args \
    -p input_topic:=/imu/data \
    -p output_topic:=/imu/data_noisy \
    -p msg_type:=Imu \
    -p fault_type:=noise \
    -p noise_std_dev:=0.15
```

### Scenario 3: LiDAR Impulse Dust & Atmospheric Backscatter
Inject salt-and-pepper dropouts and Gaussian distance jitter into 2D planar LiDAR:
```bash
ros2 run ros_fault_injector injector_node \
    --ros-args \
    -p input_topic:=/scan \
    -p output_topic:=/scan_noisy \
    -p msg_type:=LaserScan \
    -p fault_type:=noise \
    -p noise_std_dev:=0.04 \
    -p salt_pepper_prob:=0.05
```

---

## 🧪 Verification & Empirical Test Suite

Run the automated test suite verifying dynamic message type resolution, quaternion $S^3$ projection, and LiDAR range bounding:

```bash
uv run --with pytest pytest -o pythonpath=. test/ -v
```

```
============================= test session starts =============================
platform win32 -- Python 3.11.15, pytest-9.2.0, pluggy-1.7.0
rootdir: C:\Users\star\Downloads\freeapps\ros-fault-injector
collecting ... collected 3 items

test/test_injector_node.py::test_msg_type_resolution PASSED              [ 33%]
test/test_injector_node.py::test_pose_noise_injection PASSED             [ 66%]
test/test_injector_node.py::test_laserscan_noise_injection PASSED        [100%]

============================== 3 passed in 0.07s ==============================
```

---

<details>
<summary>📋 Machine-Readable Metadata (Schema.org JSON-LD)</summary>

```json
{
  "@context": "https://schema.org",
  "@type": "SoftwareSourceCode",
  "name": "ros-fault-injector",
  "version": "1.0.0",
  "description": "Chaos engineering framework for ROS 2 robotics stacks, injecting deterministic and stochastic latency, Poisson message drops, 6-DOF IMU Gaussian noise, and quaternion Lie algebra perturbations.",
  "author": {
    "@type": "Person",
    "name": "Engineer Abdullah Bin Zafar",
    "email": "abz.king.1.9.2003@gmail.com",
    "affiliation": {
      "@type": "EducationalOrganization",
      "name": "University of Engineering and Technology (UET) Lahore"
    },
    "sameAs": "https://github.com/EngineerAbdullahBinZafar"
  },
  "license": "https://opensource.org/licenses/MIT",
  "codeRepository": "https://github.com/EngineerAbdullahBinZafar/ros-fault-injector",
  "programmingLanguage": "Python",
  "applicationCategory": "RoboticsChaosEngineering",
  "operatingSystem": "Linux, macOS, Windows",
  "keywords": [
    "ros2",
    "chaos-engineering",
    "fault-injection",
    "robotics-reliability",
    "poisson-distribution",
    "sensor-noise",
    "imu-simulation",
    "quaternion-normalization",
    "humble",
    "jazzy"
  ]
}
```

</details>

---

## 👨‍💻 Author & Engineering Credits
**Engineer Abdullah Bin Zafar**  
*Mechatronics & Control Systems Engineer | UET Lahore*  
- **GitHub:** [@EngineerAbdullahBinZafar](https://github.com/EngineerAbdullahBinZafar)  
- **Email:** abz.king.1.9.2003@gmail.com  
- **Flagship R&D:** [Automotive Chassis CBF](https://github.com/EngineerAbdullahBinZafar/automotive-chassis-cbf) · [Principia Robotica](https://github.com/EngineerAbdullahBinZafar/principia-robotica)
