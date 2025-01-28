OctoFan Controller with PID
===========================

This project implements a Python-based fan control for Octominer X8/X12 Ultra server using a PID (Proportional-Integral-Derivative) algorithm to adjust fan speeds based on temperature measurements. It is designed to run on Ubuntu 24.04 and provides an automatic start feature via `systemd`.

Features
--------

*   Dynamically adjusts fan speeds based on temperature.
*   Configurable PID parameters (`Kp`, `Ki`, `Kd`).
*   Supports setting minimum and maximum fan speeds.
*   Configurable delay between logs.
*   Logs detailed PID computations for debugging.
*   Automatically starts at boot using `systemd`.

How the PID Controller Works
----------------------------

A PID controller is a feedback mechanism used to adjust a system's behavior towards a desired setpoint. It consists of three components:

*   **Proportional (P):** Adjusts the fan speed proportionally to the temperature error (difference between measured and target temperature).
*   **Integral (I):** Accounts for past errors by summing them over time, helping to eliminate persistent errors.
*   **Derivative (D):** Predicts future errors based on the rate of change, providing stability to the system.

By combining these components, the controller ensures that the fan speed reacts appropriately to temperature changes while minimizing overshooting and oscillations.

Prerequisites
-------------

1.  Python 3.8 or later.
2.  A Linux system (tested on an Ubuntu 24.04 system).


Installation
------------

### 1\. Clone the repository

    git clone git@github.com:Astral0/octofan_controller.git
    cd octofan_controller
        

### 2\. Copy the project files to `/opt`

Move the project files to the appropriate directory:

    sudo mkdir -p /opt/octofan_controller/
    sudo cp octofan_control_server_pid.py config.ini /opt/octofan_controller/


### 3\. Create a `systemd` service


Copy service file:

    sudo cp octofan_control.service /etc/systemd/system/


Or create a `systemd` service file to run the script at boot. Use the following commands:

    sudo nano /etc/systemd/system/octofan_control.service
        

Add the following configuration:

    [Unit]
    Description=Fan Control Server with PID
    After=network.target
    
    [Service]
    ExecStart=/usr/bin/env python3 /opt/octofan_controller/octofan_control_server_pid.py --target-temp 25 --vitesse-max 200
    WorkingDirectory=/opt/octofan_controller/
    Restart=always
    RestartSec=5
    WatchdogSec=20s
    User=root
    Environment=PYTHONUNBUFFERED=1
    StandardOutput=append:/var/log/octofan_control.log
    StandardError=append:/var/log/octofan_control.log

    [Install]
    WantedBy=multi-user.target
        

Save and reload `systemd`:

    sudo systemctl daemon-reload
    sudo systemctl enable octofan_control.service
    sudo systemctl start octofan_control.service

        
Logs
----

The script provides detailed logs for debugging. You can view real-time logs with:

    journalctl -u octofan_control.service -f
        
or

    tail -f /var/log/octofan_controller.log


## Usage

You can run the `octofan_control_server_pid.py` script manually or integrate it into a `systemd` service. For example, to start it manually:

```bash
python3 /opt/octofan_controller/octofan_control_server_pid.py --target_temp 25 --max_speed 200
```

### Command Line Parameters

| Argument               | Default Value   | Description                                                                 |
|------------------------|-----------------|-----------------------------------------------------------------------------|
| \--target_temp         | 40              | Target temperature in °C.                                                  |
| \--min_speed           | 40              | Minimum fan speed (from 0 to 255).                                          |
| \--max_speed           | 255             | Maximum fan speed (from 0 to 255).                                          |
| \--interval            | 10.0            | Interval, in seconds, between updates.                                      |
| \--kp                  | 2.0             | Proportional gain of the PID controller.                                    |
| \--ki                  | 0.5             | Integral gain of the PID controller.                                        |
| \--kd                  | 1.0             | Derivative gain of the PID controller.                                      |
| \--margin              | 2.0             | Temperature margin below the target to increase speed (°C).                 |
| \--fan-controller-path | fan_controller_cli | Path to the `fan_controller_cli` executable.                              |
| \--log-delay           | 600             | Delay, in seconds, between summary logs.                                    |
| \--verbose             | False           | Enables detailed log output.                                               |

For more information on configuration or `systemd` integration, refer to the associated documentation.


Uninstallation
--------------

To remove the project:

    sudo systemctl stop octofan_control.service
    sudo systemctl disable octofan_control.service
    sudo rm /etc/systemd/system/octofan_control.service
    sudo rm -rf /opt/octofan_controller/
        

Contributing
------------

Feel free to submit issues or pull requests to improve the project!

License
-------

This project is licensed under the MIT License.
