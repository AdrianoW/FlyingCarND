# WRITEUP
This doc describes the solutions given to the rubrics to the final project of the flying car nanodegree

### Step 1: Sensor Noise
The data was collected and the standard deviation was done using a Excel spreadsheet. File `6_Sensornoise.txt.` was changed with:

```C++
MeasuredStdDev_GPSPosXY = 0.73
MeasuredStdDev_AccelXY = 0.513747003
```

### Step 2: Attitude Estimation
In file `QuadEstimatorEKF.cpp`, changed the function `UpdateFromIMU()` to:

```C++ 
Quaternion<float> attitude = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, ekfState(6));
attitude.IntegrateBodyRate(gyro, dtIMU);
V3D a = attitude.ToEulerYPR();
float predictedPitch = a.y;
float predictedRoll = a.z;
ekfState(6) = a.x;

```

### Step 3: Prediction Step
In `QuadEstimatorEKF.cpp`, implemented the state prediction step in the `PredictState()`:

```C
// convert the acceleration to inertial frame
V3F add_accel = attitude.Rotate_BtoI(accel*dt);

// create the updated G, using the current speed and the acceleration
VectorXf g = curState;
g(0) = g(0) + g(3)*dt;
g(1) = g(1) + g(4)*dt;
g(2) = g(2) + g(5)*dt;
g(3) = g(3) + add_accel.x;
g(4) = g(4) + add_accel.y;
g(5) = g(5) - CONST_GRAVITY*dt + add_accel.z;

predictedState = g;

```

then `GetRbgPrime()`:

```C
// make the RbgPrime Matrix
RbgPrime(0,0) = -cos(pitch) * sin(yaw);
RbgPrime(0,1) = -sin(roll) * sin(pitch) * sin(yaw) - cos(roll) * cos(yaw);
RbgPrime(0,2) = -cos(roll) * sin(pitch) * sin(yaw) + sin(roll) * cos(yaw);
RbgPrime(1,0) =  cos(pitch) * sin(yaw);
RbgPrime(1,1) =  sin(roll) * sin(pitch) * sin(yaw) - cos(roll) * sin(yaw);
RbgPrime(1,2) =  cos(roll) * sin(pitch) * cos(yaw) + sin(roll) * sin(yaw);
```

later `Predict()`:

```C
gPrime(0,3) = dt;
gPrime(1,4) = dt;
gPrime(2,5) = dt;
gPrime(3, 6) = (RbgPrime(0) * accel).sum() * dt;
gPrime(4, 6) = (RbgPrime(1) * accel).sum() * dt;
gPrime(5, 6) = (RbgPrime(2) * accel).sum() * dt;

ekfCov = gPrime * (ekfCov * gPrime.transpose()) + Q
```

Finally, changed the parameters in file `QuadEstimatorEKF.txt`:

```
QPosXYStd = .075
QVelXYStd = .2
```

### Step 4: Magnetometer Update
Tune the parameter *QYawStd* `QuadEstimatorEKF.txt`:

```
QYawStd = .07
```
Then implemented magnetometer update in the function `UpdateFromMag()`.

```C
hPrime(0, 6) = 1;
zFromX(0) = ekfState(6);

// check the difference between the measured and estimated yaw
float diff = magYaw - zFromX(0);

// normalize the difference
if (diff > F_PI){
    zFromX(0) += 2 * F_PI;
}
else if (diff < - F_PI) {
    zFromX(0) -= 2*F_PI;
}
```

### Step 5: Closed Loop + GPS Update

Tuned the process noise model in `QuadEstimatorEKF.txt` :


Implemented the EKF GPS Update in the function `UpdateFromGPS()`.

```C
hPrime(0,0) = 1;
hPrime(1,1) = 1;
hPrime(2,2) = 1;
hPrime(3,3) = 1;
hPrime(4,4) = 1;
hPrime(5,5) = 1;

zFromX(0) = ekfState(0);
zFromX(1) = ekfState(1);
zFromX(2) = ekfState(2);
zFromX(3) = ekfState(3);
zFromX(4) = ekfState(4);
zFromX(5) = ekfState(5);
```


### Step 6: Adding Your Controller

I have compared my controller with the one provided and they were very similar.

The parameters that I had used to tune were already passing the test 6. 


