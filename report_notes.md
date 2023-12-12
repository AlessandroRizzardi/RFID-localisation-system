# Report Intelligent distribuited system

## Abstract
> Here: *Abstract*

---

##  I Introduction & Problem descriotion
> Here: *Introduzione al problema di identificazione di un tag RFID*

The inital problem we had was to find th eposition of an RFID tag using an RFID reader. RFID stands for Radio-Frequency Identification aand it is a technology that uses radio waves to identify and track object. An RFID system consist of two main components: an RFID tag and a RFID reader.

The RFID reader is a device that generate radio signals and communicates with the RFID tag. It emits radio waves in a specific frequency range and detects the signals reflected by the passive RFID in its vicinity. The tag is define as "passive" does not make use of any battery but it activates only when it receives the reader's signal.

Due to the spatial periodicity of the phase, distances wich differ by a multiple of $\lambda/2$ (where $\lambda$ is the wavelength of the signal), give rise to the same phase reading. This ambiguity in the phase measurment make impossible to directly recover the distance between the tag and the reader.

In this project we tried to implement an algorithm able to use a swarm of mobile robot with an RFID reader to identify tag position.

---

## II System model
> Here: *descrizione del robot differenziale, encoder e RFID reader*

The mobile robots used are based on the *differential drive model*

### A. Dynamics of the robot's model
$$
\begin{align}
    \dot{x} &= v \, cos(\theta) \\
    \dot{y} &= v \, sin(\theta) \\
    \dot{\theta} &= \omega
\end{align}

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
\begin{align}
    x_{k+1} &= x_k +  v_k \, cos(\theta_k) \, \Delta t \\
    y_{k+1} &= y_k +  v_k \, sin(\theta_k) \, \Delta t \\ 
    \theta_{k+1} &= \theta_k + \omega \, \Delta t
\end{align}
$$

### B. Odometry of the robot's model
If we approximate the angular velocity of the wheels as:

$$ \omega_R = \frac{d}{dt} \phi_R \simeq \frac{\Delta \phi_R}{\Delta t}$$
$$ \omega_R = \frac{d}{dt} \phi_R \simeq \frac{\Delta \phi_R}{\Delta t}$$

If we measure the rotation of the wheel ($\Delta \phi_R^{meas},\Delta \phi_L^{meas} $), we can recover the robot pose:

$$
\begin{align}
    x_{k+1} &= x_k +  \frac{R}{2} \,(\Delta \phi_R^{meas}+\Delta \phi_L^{meas}) \, cos(\theta_k)\\
    y_{k+1} &= y_k +  \frac{R}{2} \,(\Delta \phi_R^{meas}+\Delta \phi_L^{meas}) \, sin(\theta_k)\\ 
    \theta_{k+1} &= \theta_k + \frac{R}{d} \,(\Delta \phi_R^{meas}+\Delta \phi_L^{meas})
\end{align}

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
\begin{align}
    x_{k+1} &= x_k +  \frac{1}{2} \,(u_R^{meas}+u_L^{meas}) \, cos(\theta_k) \\
    y_{k+1} &= y_k +  \frac{1}{2} \,(u_R^{meas}+u_L^{meas}) \, sin(\theta_k)\\ 
    \theta_{k+1} &= \theta_k + \frac{1}{d} \,(u_R^{meas}-u_L^{meas})
\end{align}
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
u_R^{meas} =u_R + \eta_R
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
> Here: *Multi-Hypotesis Kalamn filter per trovare tag e poi soluzione distribuita con un network di robot per migliorare la stima*

### A. Tag position identification with MHEKF
The two parameter to identify the tag position with respect to the robot's one are the distance robot-tag $\rho$ and the bearing angle $\beta$.

$$
\begin{align}
    \rho &= \sqrt{(x_r - x_T)^2 + (y_r - y_T)^2} \\
    \beta &= \theta_r - \text{atan2}(y_T - y_r, x_T - x_r)
\end{align}
$$

Knowing the wheels displacements $u_{R,k}$, $u_{L,k}$ is possible, after discretization, to describe the dynamics of the variable $\rho_k$ and $\beta_k$:

$$
\begin{align}
    \rho_{k+1} &= \rho_k - u_k \, \cos{(\beta_k)}\\
    \beta_{k+1} &= \beta_k + \omega_k + \frac{u_k}{\rho_k} \, \sin{(\beta_k)} 
\end{align}
$$

We can consider the measurment model as:

$$
\begin{equation}
    z_k = 
    \begin{pmatrix}
        1 & 0
    \end{pmatrix}
    \begin{pmatrix}
        \rho_k \\ \beta_k
    \end{pmatrix}
    + \eta_k
\end{equation}
$$

We can porcede with the initialization of an Extended Kalman Filter (EKF) that makes use of the dynamics the presented dynamic and measurment model.

So let: 

$$
\begin{align}
    \hat{\rho}_0 &= z_0 \\
    \hat{\beta} &= 0
\end{align}
$$
and:

$$
\begin{equation}
    P_0 = 
    \begin{pmatrix}
        \sigma_{\rho} & 0 \\
        0             & (\pi/3)^2
    \end{pmatrix}
\end{equation}
$$

We can know compute the prediction step:

$$
\begin{align}
    \hat{\rho}_{k+1}^- &= \rho_k - u_k^{meas} \, \cos{(\beta_k)}\\
    \hat{\beta}_{k+1}^- &= \beta_k + \omega_k + \frac{u_k^{meas}}{\rho_k} \, \sin{(\beta_k)} 
\end{align}
$$

$$
 \begin{equation}
    P_{k+1}^- = F_k \, P_k \, F_k^T + W_k \, Q_k \, W_k^T 
\end{equation}
$$

where $F$ and $W$ are the jacobian matrix of the state dynamics.




### B. Distribuited algorithm


---

## IV Implementation
> Here: *Descrizione dei parametri del modello del robot, le varie varianze utilizzate e magari una breve descrizione dell'algoritmo di gestione delle varie fasi della simulazione*

---

## V Experimental result
> Here: *Risultati sperimentali dellle simulazioni*

---

## VI Conclusion
Here: *Ragionamenti sui risultati ottenuti, limiti del modello e possibili migliorie che si potrebbero fare*
