# COMP0217 Coursework Project

All code files are under the code/ folder. To run the EKF and get metrics for each task, run main.m. The plot_trajectory line can also be uncommented to plot the trajectory of the robot. Passing `true` as the last argument to the function will show an animation of the vehicle.

EKF.m is the file that contains the EKF function that has the same signature as requested by the coursework brief. EKF_update.m contains the function that is called repeatedly by EKF.m to update the state and covariance matrices. The other files are helper functions that are used by the EKF function. The majority of them are used for calibration.

Files beginning with calibrate* are used to calibrate the sensors; files beginning with find* are used to find calibration parameters, such as offsets.
