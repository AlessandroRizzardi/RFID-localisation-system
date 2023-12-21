# Report Intelligent distribuited system

## Index

[Abstract](#abstract)<br>
[Introduction](#i-introduction--problem-description)<br>
[System models](#ii-system-model)
- [Robot dynamics model](#a-dynamics-of-the-robots-model)<br>
- [Robot odometry model](#b-odometry-of-the-robots-model)<br>
- [Rfid sensor model](#c-phases-sensor)<br>

[Proposed solution](#iii-proposed-solution)<br>
[Implementation](#iv-implementation)<br>
[Experimental results](#v-experimental-result)<br>
[Conclusions](#vi-conclusion)<br>


## Abstract
> Here: *Abstract*

---

##  I Introduction & Problem description

The inital problem we had was find th eposition of an RFID tag using an RFID reader. RFID stands for Radio-Frequency Identification aand it is a technology that uses radio waves to identify and track object. An RFID system consist of two main components: an RFID tag and a RFID reader.

The RFID reader is a device that generate radio signals and communicates with the RFID tag. It emits radio waves in a specific frequency range and detects the signals reflected by the passive RFID in its vicinity. The tag is define as "passive" does not make use of any battery but it activates only when it receives the reader's signal.

Due to the spatial periodicity of the phase, distances wich differ by a multiple of $\lambda/2$ (where $\lambda$ is the wavelength of the signal), give rise to the same phase reading. This ambiguity in the phase measurment make impossible to directly recover the distance between the tag and the reader.

In this project we tried to implement an algorithm able to use a swarm of mobile robot with an RFID reader to identify tag position.

---

## II System model
> Here: *descrizione del robot differenziale, encoder e RFID reader*

The mobile robots used are based on the *differential drive model*

### A. Dynamics of the robot's model
$$
\begin{aligned}
    \dot{x} &= v \, cos(\theta) \\
    \dot{y} &= v \, sin(\theta) \\
    \dot{\theta} &= \omega
\end{aligned}

$$

We can express how the angular velocity of the wheels are related to the angular and linear velocity of the robot:

$$
\begin{equation}
    \begin{pmatrix}
        v \\
        \omega
    \end{pmatrix}
    = R
    \begin{pmatrix}
        1/2 & 1/2 \\
        1/d & 1/d
    \end{pmatrix}
    \begin{pmatrix}
        \omega_R \\
        \omega_L
    \end{pmatrix}
\end{equation}
$$

where $R$ is the wheels radius and $d$ is the length of the robot(c'è un modo migliore di dirlo)

If we discretize with $\Delta t$ (non mi viene il nome di sta roba) using Euler method we obtained the discretized system:
$$
\begin{aligned}
    x_{k+1} &= x_k +  v_k \, cos(\theta_k) \, \Delta t \\
    y_{k+1} &= y_k +  v_k \, sin(\theta_k) \, \Delta t \\ 
    \theta_{k+1} &= \theta_k + \omega \, \Delta t
\end{aligned}
$$

### B. Odometry of the robot's model
If we approximate the angular velocity of the wheels as:

$$ \omega_R = \frac{d}{dt} \phi_R \simeq \frac{\Delta \phi_R}{\Delta t}$$
$$ \omega_R = \frac{d}{dt} \phi_R \simeq \frac{\Delta \phi_R}{\Delta t}$$

If we measure the rotation of the wheel ($\Delta \phi_R^{meas},\Delta \phi_L^{meas} $), we can recover the robot pose:

$$
\begin{aligned}
    x_{k+1} &= x_k +  \frac{R}{2} \,(\Delta \phi_R^{meas}+\Delta \phi_L^{meas}) \, cos(\theta_k)\\
    y_{k+1} &= y_k +  \frac{R}{2} \,(\Delta \phi_R^{meas}+\Delta \phi_L^{meas}) \, sin(\theta_k)\\ 
    \theta_{k+1} &= \theta_k + \frac{R}{d} \,(\Delta \phi_R^{meas}+\Delta \phi_L^{meas})
\end{aligned}

$$

We can express the mesurment as displacement of the wheels:
$$
u_R = R \, \Delta \phi_R^{meas}
$$

$$
u_L = R \, \Delta \phi_L^{meas}
$$

So the odometry becomes:

$$
\begin{aligned}
    x_{k+1} &= x_k +  \frac{1}{2} \,(u_R^{meas}+u_L^{meas}) \, cos(\theta_k) \\
    y_{k+1} &= y_k +  \frac{1}{2} \,(u_R^{meas}+u_L^{meas}) \, sin(\theta_k)\\ 
    \theta_{k+1} &= \theta_k + \frac{1}{d} \,(u_R^{meas}-u_L^{meas})
\end{aligned}
$$

If we consider a real-worl situation on our measurment we will have a certain amount of uncertainty given by the noise in the measurment process:
$$
\phi^{meas} = \phi + \eta
$$

If instead of wheel rotation we consider the wheel displacement the measurment model becomes:

$$
u_R^{meas} =u_R + \eta_R 
$$

$$
u_L^{meas} =u_L + \eta_L
$$

where the noise terms are 0-mean gaussian random variable with variance:
$$
\sigma_R^2 = K_R \, |u_R|
$$

$$
\sigma_L^2 = K_L \, |u_L|
$$
### C. Phase's Sensor

Given an EM signal with frequency $f$, the phase of this signal over time is computed as:

$$
\varphi = \omega \, \cdot \, t
$$

where $\omega = 2 \pi f$.

Supposing the signal travel from reader(EM source) to the tag and back to th reader the distance of between them will be:

$$
d = \frac{1}{2} \,c \cdot t_f
$$

with $c$ the speed of light and $t_f$ the time of flight.

So:

$$
d = \frac{1}{2} \,  c \,  \frac{\varphi}{\omega} = \frac{1}{2} \,  \frac{c \, \varphi}{2 \pi f} = \frac{1}{2} \, \frac{\lambda}{2 \pi} \, (2 n \pi + \Delta \varphi) = \frac{1}{2} \, \lambda \,(n + \Delta n)
$$

So if we know $\varphi$ whe can compute $\Delta n$:

$$
\Delta n = \varphi \,\, mod \,\, 2 \pi
$$

The RFID reader measure $\Delta \varphi$ , so we actually can not directly measure $d$.

---

## III Proposed solution

### A. Tag position identification with MHEKF
The two parameter to identify the tag position with respect to the robot's one are the distance robot-tag $\rho$ and the bearing angle $\beta$.

$$
\begin{aligned}
    \rho &= \sqrt{(x_r - x_T)^2 + (y_r - y_T)^2} \\
    \beta &= \theta_r - \text{atan2}(y_T - y_r, x_T - x_r)
\end{aligned}
$$

Knowing the wheels displacements $u_{R,k}$, $u_{L,k}$ is possible, after discretization, to describe the dynamics of the variable $\rho_k$ and $\beta_k$:

$$
\begin{aligned}
    \rho_{k+1} &= \rho_k - u_k \, \cos{(\beta_k)}\\
    \beta_{k+1} &= \beta_k + \omega_k + \frac{u_k}{\rho_k} \, \sin{(\beta_k)} 
\end{aligned}
$$

What we want to do is to recover the tag position using only odometry measurment and phase measurment of the RFID signal. The best solution to pursuit this objective is to use a Kalman Filter to estimate our system's state  $\xi = (\rho_k \quad \beta_k \quad x_k \quad y_k \quad \theta_k)^T$ 

We can consider the measurment model we are going to use to update the estimate is the following:

$$
\begin{equation}
    z_k = 
    \begin{pmatrix}
        1 & 0 & 0 & 0 & 0
    \end{pmatrix}
    \xi
    + \eta_k
\end{equation}
$$

The mai problem we are facing as explained in section II is that we can not directly measure the distance $\rho$ due to the phase ambiguity. So the solution to this problem is to perform multiple kalman filters in parallel in which we consider as starting conditions different possibilities based on the lengthwave of the signal.


We can porcede with the initialization of an Extended Kalman Filter (EKF) that makes use of the dynamics the presented dynamic and measurment model.

So let: 

$$
\begin{aligned}
    \hat{\rho}_0 &= dubbio \\
    \hat{\beta} &= 0 \\
    \hat{x}_0 &= x_{i} \\ 
    \hat{y}_0 &= y_{i} \\
    \hat{\theta}_0 &= \theta_{i} 
\end{aligned}
$$
and:

$$
\begin{equation}
    P_0 = 
    \begin{pmatrix}
        \sigma_{\rho} & 0 & 0 & 0 & 0 \\
        0             & (\pi/3)^2 & 0 & 0 & 0 \\
        0 & 0 & cov(x_{i}^2) & cov(x_i \, y_i) & cov(x_i \, \theta_i) \\
        0 & 0 & cov(y_i \, x_i) & cov(y_{i}^2) & cov(y_i \, \theta_i) \\
        0 & 0 & cov(\theta_i \, x_i) & cov(\theta_i \, y_i) & cov(\theta_{i}^2) \\
    \end{pmatrix}
\end{equation}
$$

We can know compute the prediction step:

$$
\begin{aligned}
    \hat{\rho}_{k+1}^- &= \hat{\rho}_k - u_k^{meas} \, \cos{(\beta_k)}\\
    \hat{\beta}_{k+1}^- &= \hat{\beta}_k + \omega_k + \frac{u_k^{meas}}{\rho_k} \, \sin{(\beta_k)} \\
    \hat{x}_{k+1}^- &= \hat{x}_k +  \frac{1}{2} \,u_k^{meas} \, cos(\theta_k) \\
    \hat{y}_{k+1}^- &= \hat{y}_k +  \frac{1}{2} \,u_k^{meas} \, sin(\theta_k)\\ 
    \hat{\theta}_{k+1}^- &= \hat{\theta}_k + \frac{1}{d} \,\omega_k^{meas}
\end{aligned}
$$

$$
 \begin{equation}
    P_{k+1}^- = F_k \, P_k \, F_k^T + W_k \, Q_k \, W_k^T 
\end{equation}
$$

where $F$ and $W$ are the jacobian matrix of the state dynamics:


$$
\begin{equation}
    F_k = 
    \begin{pmatrix}
        1 & u_k^{meas}\,\sin{(\hat{\beta}_k)} & 0 & 0 & 0 \\\\
        -\frac{u_k^{meas}}{\hat{\rho}_k^2}\, \sin{(\hat{\beta}_k)}& 1+ \frac{u_k^{meas}}{\hat{\rho}_k}\cos{(\hat{\beta}_k)} & 0 & 0 & 0 \\\\
        0 & 0 & 1 & 0 & - u_k^{meas}\, \sin{\hat{\theta}_k} \\\\
        0 & 0 & 0 & 1 & u_k^{meas}\, \cos{\hat{\theta}_k} \\\\
        0 & 0 & 0 & 0 & 1 \\
    \end{pmatrix}
\end{equation}
$$

$$
\begin{equation}
    W_k = 
    \begin{pmatrix}
        - \frac{1}{2} \, \cos{\hat{\beta}_k} & - \frac{1}{2} \, \cos{\hat{\beta}_k} \\
        \frac{1}{d} + \frac{1}{2\hat{\rho}_k}\,\sin{(\hat{\beta}_k)} &  \frac{1}{d} + \frac{1}{2\hat{\rho}_k}\,\sin{(\hat{\beta}_k)} \\
        \frac{1}{2}\,cos{(\hat{\theta}_k)} & \frac{1}{2}\,cos{(\hat{\theta}_k)} \\
        \frac{1}{2}\,sin{(\hat{\theta}_k)} & \frac{1}{2}\,cos{(\hat{\theta}_k)} \\
        \frac{1}{d} & - \frac{1}{d}
    \end{pmatrix}
\end{equation}
$$

Being $z_{k+1} = \rho_{k+1} + \eta_{\rho}$ the update will be compute as:

$$
    \begin{equation}
        \begin{pmatrix}
            \hat{\rho}_{k+1} \\
            \hat{\beta}_{k+1}
        \end{pmatrix}
        =
        \begin{pmatrix}
            \hat{\rho}_{k}^- \\
            \hat{\beta}_{k}^-
        \end{pmatrix}
        +
        K_{k+1}\,(z_{k+1} - \hat{\rho}_{k+1}^-)
    \end{equation}
$$

where:

$$
    \begin{equation}
        K_{k+1} = (H \, P_{k+1}^- \, H^T + \sigma_\rho)^{-1}\,P_{k+1}^- \, H^T
    \end{equation}
$$

We said that what we measure is the phase $\Delta \varphi$ of the RFID signal, so the innovation term is actually $ \varphi_{k+1} - \hat{\varphi}_{k+1}$, where $\hat{\varphi} = mod(-2*dubbio*\hat{\phi}_{k+1})$ is th expected phase measurment.

So the matrix $H$ becomes: ...

To determine what istance we should select at eachh step they are all weighed. the weight is initialized for all instances as:
$$
w_i = \frac{1}{n_M} \quad i = 1, ... \, , \, n_M 
$$

and at every steps they re computed as:

$$
    w_i = w_i \, e^{-\frac{1}{2}\,(H \, P \, H^T + \sigma_{\phi})^{-1}\, (\varphi - \hat{\varphi})^2} 
$$

### B. Distribuited algorithm
In order to improve the estimate of the single robot we can implemented a swarm algorithm in which each node (robot) trasmits to all the other nodes in its communication range. Under some dome connectivity properties of the robotic network the node will have alll the network's data.

Heach node can compute a measurment of the tag position:

$$
\begin{equation}
    z 
    =
    \begin{pmatrix}
        1 & 0 \\
        0 & 1
    \end{pmatrix}
    \begin{pmatrix}
        x_T \\
        y_T
    \end{pmatrix}
    + \eta_T
\end{equation}
$$

Each robot compute the estimate of the tag position as :

$$
    \begin{aligned}
        x_T &= x_R + \rho \, \cos{(\theta_R - \beta)} \\
        y_T = y_R + \rho \, \sin{(\theta_R - \beta)}
    \end{aligned}
$$

In order to compute $ Cov(\eta_T):

$$
    \begin{equation}
        Cov(\eta_T) = J \, P \, J^T 
    \end{equation}
$$

where $P$ is the covariance matrix of the state $\xi = (\rho \beta x_R y_R \theta)$, while $J$ is the Jacobian matrix of the poistion of the tag with reference to the $\xi$ state:

$$
\begin{equation}
    J = 
    \begin{pmatrix}
        \cos{(\hat{\theta} - \hat{\beta})}& \hat{\rho} \, \sin{((\hat{\theta} - \hat{\beta}))} & 1 & 0 & - \hat{\rho} \, \sin{(\hat{\theta} - \hat{\beta})} \\
        \sin{(\hat{\theta} - \hat{\beta})} & - \hat{\rho} \, \cos{(\hat{\theta} - \hat{\beta})} & 0 & 1 & \hat{\rho} \, \cos{(\hat{\theta} - \hat{\beta})}
    \end{pmatrix}
\end{equation}

$$



The estimate of each node at a certain time will be:

$$
\begin{equation}
    \begin{pmatrix}
       \hat{x}_T \\
        \hat{y}_T
    \end{pmatrix} 
    = F_i^{-1} \, a_i
\end{equation}
$$

where:

$$
\begin{aligned}
    F_i &= H \, Cov(\eta_T)^{-1} \, H^T\\
    a_i &= H^T Cov(\eta_T) \, 
    \begin{pmatrix}
       \hat{x}_{i,T} \\
        \hat{y}_{i,T}
    \end{pmatrix} 
\end{aligned}
$$

We can implment a consensiu algorithm in which we are going to mìexchange a sufficiently high amount of messages between nodes in order to ensure the convergence to a consensus equilibrium.

$$
\begin{aligned}
    F_i(k+1) &= F_i(k) + \sum_{j =1}^{n} \, \frac{1}{1 + d_{max}} (F_j(k) - F_i(k)) \\
    a_i(k+1) &= a_i(k) + \sum_{j =1}^{n} \, \frac{1}{1 + d_{max}} (a_j(k) - a_i(k)) \\
\end{aligned}
$$

So we will have that:

$$
\begin{equation}
    \begin{pmatrix}
       \hat{x}_{T} \\
        \hat{y}_{T}
    \end{pmatrix}
    = 
    \lim_{k \to \infty} F_i(k)^{-1} \, a_i(k)
\end{equation}
$$




---

## IV Implementation
To prove the goodness of the algortihm we build a simulation environment of $100 m^2$. The mobile robot as been model as a differential drive robot with distance between wheels of $50 cm$ and radius of the wheel of $20 cm$.
The environment has been set such that the robot is unaware of the tag position and randomly search around the map generating random target point. If one of the robot is able to find the tag entering in the range of the RFID signal it comunicates its position to the other robot that immediately point towards the communivated location.

The RFID signal has a frequancy of $867 \cdot 10^6 \, Hz$, that means a wave length around $35 \, cm$. The signal range has been set to $2 m$, meaning that every robot has to compute $12$ Kalman Filter to determine in which cycle is the tag.

Once in the range every robot has an estimate of the tag (in the distribuited system the consensus will bring to a common value): to continue improving th e estimate the robot must remain inside the tag as  much as possible so it can use its knowladge of the tag position to try to move around it without exceeding the range limit.

In order to move in the environment towards the computed target point the controls of the robots use a proportional controller such that:

$$
    \begin{aligned}
        \theta_d &= atan(\frac{y_d - y_R}{ x_d - x_R}) \\
        e_{\theta} &= atan(\frac{\sin{(\theta_d - \theta_R)}}{\cos{(\theta_d - \theta_R)}}) \\
        \delta &= \sqrt{(y_d - y_R)^2 +  (x_d - x_R)^2}\\
        \omega &= K_{P,\omega} \cdot e_{\theta}\\
        v &= K_{P,v} \cdot \delta
    \end{aligned}
$$

We also impose that the linear velocity $v$ of the robot can not exceed $2 m/s$


The distribuited algorith take in ccount all the robot sinside the range (i.e. all the nes that have an estimate of teh tag position) and perform a consesus algorith with an exchange of $50$ messages.

---

## V Experimental result

In order to be able to describe the accuracy and precision of the moedel we run some Monte-Carlo simulation. In these simulation we checked different condition regarding the odometry error and the error in the phase measurment. Moreover we checked ho the number of robot in the swarm impacts the final results

---

## VI Conclusion
The initial goal of finding the tag has been succefully reached and thanks to the distribuited approach we were able to improve the actual estimate. The analysis we conduct as lead to some quantitative results that allow to esatimate the accuracy of the system respect the error of the measurments systems. 

The environment we considered was simplified in some aspects. The phase measurments usually are influenced by multiple factors like the multipath of the signal and it could have a fix offset.

The robots are free to move in environment and is not implemented in the simulation any collision avoidance scheme. Moreover this project show how the system is able to identify the tag even if the localisatin estimate of the robot is given only by the odometry, but further development in which an positioning external system is implemented will give for sure better results.

Also the scheme that the robots follow to find the tag in the room is not optimize: some algortihm of the environment analysis may be introduced in order to optimize the tag search.
