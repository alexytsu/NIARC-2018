# LSM9DS1-Drivers-for-MyRIO
Only Gyroscope is Currently Working

## Usage
Begin must be called first to activate the gyroscope, and then calibrate is a function to calculate the raw bias of the different axes. Then gyroLop can be called in a loop to constantly update the global variables gx, gy and gz. These values can also be read directly out from the VI. Integrating any of these axes (by placing gyro loop in a timed loop) and using the Integral.vi will give you the facing of the IMU in that particular axis.
