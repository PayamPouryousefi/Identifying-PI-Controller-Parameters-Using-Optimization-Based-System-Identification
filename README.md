# Identifying PI controller parameters using optimization-based system identification


This is a data analysis task which implements system identification to determine the parameters (Kc, Ti) used in a system of PI controllers for a heat exchange process with flow and tempreture control loops.


The model fits simulated PI control dynamics to operational data in order to quantify the proportional gain (Kc) and integral time (Ti) constants providing the observed controller performance.


**Overview**

The heat exchange process has:

* Flow control with pump uc
* Outlet temperature control with pump uh
* Inlet temperature control with heater up

The data includes actuator signals and process variable responses.


**PI equations**

u(k) = u(k-1) + Kc*((1 + Ts/Ti) * e(k) - e(k-1))
* u(k) is the control signal at time k calculated by the controller
* u(k-1) is the control signal from the previous time step
* Kc: Proportional gain
* Ts: Sampling time
* Ti: Integral time
* e(k): Current error
* e(k-1): Previous error


e(k) = r(k) - y(k)
* e(k) is the error at the current time step k. It's calculated as the difference between the setpoint r(k) and the process variable y(k).
* This error will be used by the PI controller to determine how much control action is needed.


**Requirements**

The simulation requires the following Python packages:

* numpy
* scipy
* matplotlib


To install requirements

```
pip install -r requirements.txt
```

Older versions may work but have not been tested. It is advised to create and activate a dedicated virtual environment for running the simulation cleanly.


For example, on Linux/macOS

```
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

On Windows
```
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
```
With the virtual environment activated, run the simulation script normally.
I was able to use miniconda 23.9.0 to run the code as well.

**Usage**

The data analysis is implemented in Python. To run:

```
python src\pi_controller.py
```

The script loads parameters, integrates the equations, and plots the results. See the code for analysis details.
