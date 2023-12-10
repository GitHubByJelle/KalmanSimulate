# Robot-Localization-Analysis: Visualization of Room Navigation with Kalman Filter and Sensor Emulation
This project is part of an group assignment for the course Autonomous Robotic Systems for my Master's in Artificial Intelligence. The project's main goals was to visualise the uncertainties robots have in real-world scenario's using sensors at stationary positons in a room. The implemented simulation contains a circular robot navigating through a room with stationary (noisy) sensors. While moving through the room, you can see the location the robot think it is in, as well as the actual location of the robot. The implementation can be used as playground to better understand the influence of sensor quality (measurement noise) and motion model uncertainty.

<p align="center" width="100%">
    <img src="images\robot-localization-analysis.gif" alt="Visualisation of the Kalman Filter" width="70%">
</p>

## Implementation Details
For AI functionality, the code leverages the following techniques/algorithms:
* Kalman Filter

The robot implementation utilized in this project is closely tied to the one employed in my other project, '[NeuralDustCollector](https://github.com/GitHubByJelle/NeuralDustCollector).'

## How to use
First install the requirements.txt
```bash
pip install -r requirements.txt
```

To see a visualisation of the Kalman Filter run `main.py`
```bash
python main.py
```

The blue lines indicated the actual path, the orange line indicates the position the robot thinks it is in. The orange circles indicate the uncertainty of the robots position.

To modify the sensor quality (measurement noise) or motion model uncertainty parameters in `KF/kalmanfilter.py` can be modified.