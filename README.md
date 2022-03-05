# nwang-EKF-sim
Python pose-EKF implementation

## Intall Dependencies
Using a new virtual env to install the packages:
`pip install -r requirements.txt`

## Run Simulation
To run the simulator, just run the following command in your terminal :
`python fast_slam.py`

## Control
Using arrow keys to control the robot

## Pose EKF
The robot's velocity is a non-linear exponential function. The EKF takes in a number of inputs; 
an estimated x and y pose, velocity, yaw angle and the rate of change of yaw. All these measurements
have noise included. 

The EKF linearises the non-linear inputs and uses a standard Kalman Filtering algorithm. The estimated pose
is drawn onto the screen as a purple circle and the error in the x and y coordinates are printed to the 
terminal. 

