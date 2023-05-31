# Object_Tracking_Kalman_Filter

III. OBJECT TRACKING
A. Literature Review For Task 3
The most crucial aspect of object detection and tracking
is the use of surveillance technologies. The first stage is to
find any object and second stage is to track it. There are
many approaches that can be used to find moving objects.
The system that depends on a pedestrian detection method was
proposed by Hong-Son [20]. A Kalman filter in combination
with a visual system to determine the location and orientation
of the vehicle in relation to its surroundings. A camera records
photographs of the area around the car and then computer
analyses the images to extract important details like lines and
corners. The position and orientation of the vehicle are then
estimated by the Kalman filter using those features [21]. In
robot vision, factors like shifting lighting conditions may make
the usual Kalman filter ineffective. The adaptive Kalman filter
performs better than the normal Kalman filter when used with
a robot vision system to track a moving object in a video series
[22].
B. Kalman Filter Algorithm
For Tracking Object Kalman Filter is used. Kalman filter is
a mathematical algorithm which uses a series of measurements
taken over time and estimates locaion of unknown objects with
accuracy which is good compared to individual measurements.
Kalman filter used to process measurements over time which
updates estimated value of the location of unknown object. The
estimation for the current time step is updated by Kalman filter
by computing a weighted average of the present estimate and
it will predict based on the past estimate [23]. It involves two
steps update and prediction step. Kalman filter predicts current
state of the system based on past state which include estima-
tion of current state and uncertainities whereas in update step
the filter uses new measurements to update the estimation of
the present step which includes correction factor that is equal
to the difference between predicted state and measurements.
Kalman filter widely used in estimation and control problems
which deals with uncertainity in real world problem [24].
In this problem kalman filter is used to estimate the coor-
dinates of a moving object based on the noisy coordinates.
The input data is set of noisy coordinates which are provided
in ’a.csv’ and ’b.csv files whereas output is an estimation of
actual real target coordinates which are provided in ’x.csv’ and
’y.csv’ files. It uses Constant Velocity motion model F with
time intervals dt = 0.2 and a Cartesian observation model H.
The covariance matrices Q and R respective to noises.
dt is sampling time which has ability to track system
dynamics accurately. A smaller dt which gives better estimates
but it will also increase computational complexity of kalman
filter
F is state transition matrix of constant velocity motion
model [23], The matrix F is given below
F =




1 dt 0 0
0 1 0 0
0 0 1 dt
0 0 0 1




which considers the object will be be moving with a constant
speed.
Q is the covariance matrix covariance matrix of motion
noise [23]. The Matrix Q is given below
Q =




0.16 0 0 0
0 0.36 0 0
0 0 0.16 0
0 0 0 0.36




This matrix values are system noise. How much errors are
there in the estimate of one variable are related to the errors in
the estimate of another variable which is given by covariance
matrix [23].
H is observation model matrix that maps the state to the
observation space [23].
H =
[1 0 0 0
0 0 1 0
]
which assumes that we can easily observe the object x and y
coordinates.
R is the covariance matrix of the observation noise. Care-
fully selecting covariance matrices Q and R is necessary to
reflect the characteristics of the system being modelled [23].
R =
[0.25 0
0 0.25
]
This matrix values are system noise.
We have chosen Constant Velocity motion model because
it is a straightforward model that works well for a wide range
of real world applications, For many moving targets, it is fair
to assume that the system velocity will remain constant across
the time interval t [23]. Cartesian observation model because
of its simplicity which will assume the observed coordinates
are directly related to the real coordinates. Covariance matrices
Q and R was already given. However, if we choose our own
covariance matrices, we would also need to take into account
the system noise level and the expected uncertainty in our state
estimation [23].
The prediction step and the correction step are the two steps
that make up the Kalman filter [24]. The Constant Velocity
motion model is applied in the prediction stage to predict the
system state at the next step. The motion model makes the
assumption that the system velocity will remain constant over
the time interval dt. The equation of prediction step is given
by
x(k|k − 1) = F · x(k − 1|k − 1) (10)
where x(k—k-1) is the predicted state at time k, x(k-1—k-
1) is the estimated state at time k-1 and F is the state transition
matrix [24].
In update step, we update our estimate of the state using the
noisy coordinates that were observed. Kalman gain K matrix
which defines how much weight to assign to the predicted state
and observed noisy coordinates which is is computed during
the update stage [24]. The equation of update step is given by:
x(k|k) = x(k|k − 1) + K · (y(k) − H · x(k|k − 1)) (11)
where x(k—k) is updated state at time k, y(k) is observed
noisy coordinates at time k, H is observation matrix and K is
Kalman gain.
Fig. 12: Kalman Filter Object Tracking Plot Graph
Prediction and update steps are performed each while it-
erating through data. Each time step, we predict the state
using the past state estimate and the system model, and then
update predicted state using noisy observation and Kalman
gain matrix [24].
In Fig 12, After running Kalman Filter, Actual values,
Estimated values and noisy values are plotted in graph. It is
noticed that estimated values are nearly close to actual value
means estimation producing accurate results.
TABLE X: Comparison of RMSE, MAE, and SD of AE for
noisy and estimated data
Metrics Noisy Estimated
Root Mean Squared Error 3.8511 0.4591
Mean Absolute Error 3.1637 0.3946
Standard Deviation of Absolute Eerror 2.2069 0.2359
Table 10, displays the Kalman filter performance. The
average absolute difference between the estimated and actual
values is measured by the Mean Absolute Error [23]. Mean
Absolute Error for estimated value is 0.3946 which is very
low compared to noisy value which is 3.1637. This indicates
estimation has produced accurate results.
Mean Absolute Error equation is given by
M eanAbsoluteError = 1
n
n∑
i=1
|, yi − ˆyi, | (12)
where,
n is the number of observations,
yi is the actual value of the i-th observation,
ˆyi is the predicted value of the i-th observation,
|.| denotes absolute value∑ is the summation operator
The spread of the errors is indicated by the Standard
Deviation of Absolute Error [23] which is 2.0858. Standard
Deviation of Absolute Error for estimated value is 0.2359
which is very low compared to noisy value which is 2.2069.
This indiactes estimation has produced accurate results. Stan-
dard Deviation of Absolute Error equation is given by
The Standard Deviation of Absolute Error (SDAE) is a
measure of the spread of the absolute errors around the mean
error. In this case, the SDAE of the estimated values is also
significantly lower than that of the noisy values, indicating that
the estimation process has produced results that are not only
more accurate but also more consistent.
SDAE =
√
√
√
√ 1
n
n∑
i=1
(|yi − ˆyi| − M AE)2 (13)
where
n is the number of samples
yi is the actual value at the i-th sample,
ˆyi is the estimated value at the i-th sample
MAE is the mean absolute error.
Root Mean Squared Error is a measurement of the square
root of the average of the squared errors [23]. Root Mean
Squared Error for estimated value is 0.4591 which is very
low compared to noisy value which is 3.8511. This indiactes
estimation has produced accurate results. Root Mean Squared
Error equation is given by
RM SE =
√
√
√
√ 1
n
n∑
i=1
(yi − ˆyi)2 (14)
where
n is the number of samples or data points
yi is the actual or true value of the i-th sample or data point
ˆyi is the estimated value of the i-th sample or data point
