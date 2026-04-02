# Kalman-Filter
This project is set up for the author himself to study Kalman Filter.
This code is taken from https://zhuanlan.zhihu.com/p/649997859
>This code implements a Kalman filter for an object moving with constant acceleration.  
>The state vector consists of position and velocity.  
>The system model (state transition matrix F and control matrix B), process noise covariance Q, observation matrix H, observation noise covariance R, and control input u (acceleration) are given.  
>The filter estimates both position and velocity at each time step using:  
>* a prediction step based on the previous estimate and the control input,  
>* an update step that fuses the prediction with noisy measurements (here, direct observations of both position and velocity).  
>The code compares the filtered estimates with the true (simulated) states and the raw measurements, and plots the results.  
# 1.Formula Derivation
## 1.1 Prediction & Update
### (1) Prediction
<p align="center">
    $$x_k^- =F \widetilde{x}  _ {k-1} + B u_ {k-1} $$  
</p>  

where  <br>
$x_k^-$ is the predicted value of $x_k$;  <br>
$\widetilde{x}$ is the optimal estimate value of $x_k$;  <br>

<p align="center">
    $$P_k^- =FP_{k-1}F^T+Q$$  
</p>

where  <br>
$P_K^-$ is the prior error covariance matrix;  <br>
$P_k$ is the posterior erreo covariance matrix;  <br>
$Q$ is the process noise;  <br>
### (2) Update
<p align="center">
    $$K=P_k^- H^T (HP_k^- H^T+R)^{-1}$$  
</p>

<p align="center">
    $$\widetilde{x}_k =x_k^- +K(z_k -Hx_k^-)$$  
</p>

<p align="center">
    $$P_k=(I-KH)P_k^-$$  
</p>

where  
$K$ is Kalman gain;

## 1.2 System Model

# 2.Results
<div align="center">
<img width="480" height="360" alt="Figure_1" src="https://github.com/user-attachments/assets/f9023d7a-bee3-4ac6-a4e5-e71590c301af" /> <br>
<img width="480" height="360" alt="Figure_2" src="https://github.com/user-attachments/assets/8d05ef19-031b-47b6-ba84-201f5e4da88d" /> <br>
</div>
