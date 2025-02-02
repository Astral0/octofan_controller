#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import re
import subprocess
import time
import sys
import os
import logging
import configparser
import socket
import usb.core
import usb.util

script_dir = os.path.dirname(os.path.realpath(__file__))

# Basic configuration
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
logger = logging.getLogger()

def notify_systemd_watchdog():
    """
    Notifies systemd that the service is healthy by directly using the notification socket.
    This must be called regularly before WatchdogSec.
    """
    watchdog_socket = os.getenv('NOTIFY_SOCKET')  # Notification socket defined by systemd
    if not watchdog_socket:
        logger.warning("NOTIFY_SOCKET is not defined. No notification sent.")
        return

    try:
        # Create a connection to the systemd UNIX socket
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        sock.connect(watchdog_socket)
        sock.sendall(b"WATCHDOG=1")  # Send the watchdog message
        sock.close()
        logger.debug("Notification sent to systemd watchdog.")
    except Exception as e:
        logger.error(f"Error notifying systemd watchdog: {e}")



class PIDController:
    def __init__(self, kp=2.0, ki=0.5, kd=1.0, setpoint=40.0, 
                 output_min=0, output_max=255):
        """
        kp, ki, kd: PID gains.
        setpoint   : Temperature setpoint (in °C).
        output_min : Minimum allowed speed (0).
        output_max : Maximum allowed speed (255).
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        
        self.output_min = output_min
        self.output_max = output_max
        
        # Internal variables for PID calculation
        self._integral = 0.0
        self._prev_error = 0.0
        self._last_time = None

    def update(self, measured_value, current_time, max_speed):
        # Calculate the error (too high temperature = positive error)
        error = measured_value - self.setpoint
        
        if self._last_time is None:
            # First iteration: simply initialize
            self._last_time = current_time
            self._prev_error = error
            return self.output_min
        
        dt = current_time - self._last_time
        if dt <= 0:
            # If time has not progressed (unlikely, but avoids division by 0)
            dt = 1e-16

        # Calculate PID terms
        proportional = self.kp * error
        self._integral += error * dt
        # Limit integral to avoid "windup"
        self._integral = max(-1000, min(self._integral, 1000))
        integral = self.ki * self._integral
        derivative = self.kd * (error - self._prev_error) / dt

        # Calculate the output
        output = proportional + integral + derivative
        output = max(self.output_min, min(output, self.output_max))

        # Detailed logs for debugging
        logger.debug(f"PID DEBUG: Measured temperature={measured_value:.2f}°C, "
                     f"Error={error:.2f}, P={proportional:.2f}, I={integral:.2f}, D={derivative:.2f}, "
                     f"Calculated speed={output:.2f}")

        # Update for the next iteration
        self._prev_error = error
        self._last_time = current_time
        
        # Clamp the output [output_min, output_max]
        output = max(self.output_min, min(output, self.output_max))
        output = min(output, max_speed)  # Limit to max_speed

        return int(output)


def usbOpenDevice(vendor, product):
    """
    Opens the USB device matching the vendor/product IDs.
    Returns a PyUSB 'dev' object if found, otherwise None.
    """
    dev = usb.core.find(idVendor=vendor, idProduct=product)
    if dev is None:
        return None

    try:
        dev.set_configuration()
    except usb.core.USBError:
        pass

    return dev


def usb_control_msg(dev, request_type, request, value, index, length=256, timeout=5000):
    """
    Sends a USB control message and returns the read data (if applicable).
    """
    try:
        data = dev.ctrl_transfer(
            request_type,  # bmRequestType
            request,       # bRequest
            value,         # wValue
            index,         # wIndex
            length,        # data or wLength
            timeout=timeout
        )
        return data
    except usb.core.USBError as e:
        logger.error(f"USB error during control transfer: {e}")
        return None


def fan_speed_update(speed, fan_ids, dev):
    """
    Updates the speed of the listed fans (fan_ids) using direct USB control.
    """
    for fid in fan_ids:
        try:
            # Command to set speed: 0x04
            usb_control_msg(dev, 0xC0, 0x04, speed, fid)
            logger.info(f"Fan {fid} successfully updated to speed {speed}.")
        except Exception as e:
            logger.error(f"Error updating fan {fid}: {e}")

    # Release USB resources
    usb.util.dispose_resources(dev)


def read_fan_ids(dev):
    """
    Reads the IDs of active fans directly via the USB interface.
    A fan is considered active if its data (RPM, PWM) is valid.

    Args:
        dev (usb.core.Device): USB device of the Fan-Controller.

    Returns:
        list: IDs of active fans (with valid RPM).
    """
    buffer_length = 256  # Typical size for USB commands
    num_fans = 12  # Total number of supported fans
    fan_ids = []

    try:
        # Read fan information
        for i in range(num_fans):
            # Read current RPM (0x03)
            crpm = usb_control_msg(dev, 0xC0, 3, 0, i, buffer_length)
            rpm_val = crpm[0] | (crpm[1] << 8)

            # Read max RPM (0x0E)
            mrpm = usb_control_msg(dev, 0xC0, 0x0E, 0, i, buffer_length)
            maxRpm = mrpm[0] | (mrpm[1] << 8)

            # Check if the fan is active (RPM > 0 and maxRPM > 0)
            if rpm_val > 0 and maxRpm > 0:
                fan_ids.append(i)
                logger.info(f"Fan {i} detected.")

    except usb.core.USBError as e:
        logger.error(f"USB error while reading fans: {e}")

    return fan_ids


def read_temperatures(dev, min_temp=0, max_temp=100):
    """
    Reads temperatures directly via the USB command.
    Ignores the first irrelevant value and returns the last three.

    Args:
        dev (usb.core.Device): USB device of the Fan-Controller.

    Returns:
        dict: Temperatures of available sensors.
    """
    buffer_length = 256  # Typical size for USB commands
    sensors = {}

    try:
        # Read temperatures
        for i in range(4):
            tbuf = usb_control_msg(dev, 0xC0, 8, 0, i, buffer_length)
            if len(tbuf) < 2:
                logger.warning(f"Insufficient data for sensor {i}.")
                continue
            
            # Calculate temperature from the first two bytes
            temp_raw = (tbuf[0] | (tbuf[1] << 8))
            
            # Convert to temperature if relevant
            if min_temp <= temp_raw <= max_temp:
                sensors[f"Temperature Sensor {i}"] = temp_raw
                logger.debug(f"Temperature Sensor {i}: {sensors[f"Temperature Sensor {i}"]}")
        
        return sensors

    except usb.core.USBError as e:
        logger.error(f"USB error while reading temperatures: {e}")
        return {}


def main():

    # Specific IDs of Octominer X8/12 Ultra Fan Controllers
    dev = usbOpenDevice(0x16c0, 0x05dc)

    if dev is None:
        logger.error("Unable to find the USB Fan Controller. Are you on an Octominer X8 or X12 Ultra server?")
        return {}

    # Load the ini file
    config = configparser.ConfigParser()
    config.read(os.path.join(script_dir, "config.ini"))

    # Default arguments from the ini file
    default_args = {
        "interval": float(config["settings"].get("interval", 10.0)),
        "target_temp": float(config["settings"].get("target_temp", 40.0)),
        "kp": float(config["settings"].get("kp", 2.0)),
        "ki": float(config["settings"].get("ki", 0.5)),
        "kd": float(config["settings"].get("kd", 1.0)),
        "min_speed": int(config["settings"].get("min_speed", 40)),
        "max_speed": int(config["settings"].get("max_speed", 255)),
        "margin": float(config["settings"].get("margin", 2.0)),
        "log_delay": int(config["settings"].get("log_delay", 600)),
    }

    # Command line arguments
    parser = argparse.ArgumentParser(description="Automatic fan control script with PID.")
    parser.add_argument("--interval", "-i", type=float, help="Interval in seconds between two readings/controls.")
    parser.add_argument("--target_temp", "-t", type=float, help="Target temperature in °C.")
    parser.add_argument("--kp", type=float, help="Proportional gain of the PID.")
    parser.add_argument("--ki", type=float, help="Integral gain of the PID.")
    parser.add_argument("--kd", type=float, help="Derivative gain of the PID.")
    parser.add_argument("--min_speed", type=int, help="Minimum fan speed.")
    parser.add_argument("--max_speed", type=int, help="Maximum fan speed.")
    parser.add_argument("--margin", type=float, help="Margin in °C below the target temperature before increasing speed.")
    parser.add_argument("--log-delay", type=int, help="Delay in seconds between summary logs.")
    parser.add_argument("--verbose", action="store_true", help="Enable detailed logs.")
    args = parser.parse_args()

    # Merge ini file arguments and command line arguments
    final_args = {
        "interval": args.interval if args.interval is not None else default_args["interval"],
        "target_temp": args.target_temp if args.target_temp is not None else default_args["target_temp"],
        "kp": args.kp if args.kp is not None else default_args["kp"],
        "ki": args.ki if args.ki is not None else default_args["ki"],
        "kd": args.kd if args.kd is not None else default_args["kd"],
        "min_speed": args.min_speed if args.min_speed is not None else default_args["min_speed"],
        "max_speed": args.max_speed if args.max_speed is not None else default_args["max_speed"],
        "margin": args.margin if args.margin is not None else default_args["margin"],
        "log_delay": args.log_delay if args.log_delay is not None else default_args["log_delay"],
    }

    # Validate final arguments
    for key, value in final_args.items():
        if value is None:
            logger.error(f"The parameter '{key}' is missing or invalid in the configuration.")
            sys.exit(1)

    # Apply detailed logs if needed
    if args.verbose:
        logger.setLevel(logging.DEBUG)

    logger.info("*** Starting fan control script (PID) ***")
    logger.info(f"Configuration: {final_args}")

    # Retrieve final arguments
    interval = final_args["interval"]
    target_temp = final_args["target_temp"]
    kp = final_args["kp"]
    ki = final_args["ki"]
    kd = final_args["kd"]
    min_speed = final_args["min_speed"]
    max_speed = final_args["max_speed"]
    margin = final_args["margin"]
    log_delay = final_args["log_delay"]

    # Instantiate PID
    pid = PIDController(
        kp=kp,
        ki=ki,
        kd=kd,
        setpoint=target_temp,
        output_min=min_speed,
        output_max=max_speed
    )

    logger.info("*** PID configuration complete. ***")

    logger.info("*** Starting fan control script (PID) ***")
    logger.info(f"Interval: {interval}s, Target temperature: {target_temp}°C, Margin: {margin}°C")
    logger.info(f"PID Gains: Kp={kp}, Ki={ki}, Kd={kd}")
    logger.info("-" * 50)

    # Read fans at startup
    fan_ids = read_fan_ids(dev)
    if not fan_ids:
        logger.error("No active fans detected. Stopping the program.")
        sys.exit(1)
    logger.info(f"Active fans detected: {fan_ids}")

    # Variables for conditional logs
    last_speed = None
    last_logged_time = time.time()

    while True:
        start_time = time.time()

        # Notify systemd that the service is alive
        notify_systemd_watchdog()

        # 1) Read temperatures
        sensors = read_temperatures(dev)
        if not sensors:
            # In case of reading failure, wait for the next cycle
            time.sleep(interval)
            continue

        # 2) Get the maximum temperature
        temp_max = max(sensors.values())

        # 3) Control logic
        if temp_max > target_temp + margin:
            # If the temperature exceeds the target + margin, activate the PID
            speed = pid.update(temp_max, current_time=start_time, max_speed=max_speed)
            if last_speed is None or speed != last_speed:
                logger.info(f"Temperature {temp_max}°C > Threshold {target_temp + margin}°C: PID activated")
        else:
            # If the temperature is below or equal to the target + margin, keep the minimum speed
            speed = min_speed
            if last_speed is None or speed != last_speed:
                logger.info(f"Temperature {temp_max}°C <= Threshold {target_temp + margin}°C: Minimum speed")

        # Limit the speed so it does not exceed the allowed bounds
        speed = max(min_speed, min(speed, max_speed))

        # 4) Update fans only if the speed changes
        if last_speed is None or speed != last_speed:
            fan_speed_update(speed, fan_ids, dev)
            logger.info(f"Fan speed updated: {speed} (previous: {last_speed})")
            last_speed = speed

        # Logs every 10 minutes
        current_time = time.time()
        if current_time - last_logged_time >= log_delay:
            logger.info(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] "
                        f"Max temperature: {temp_max:.2f}°C => Current speed: {speed}")
            last_logged_time = current_time

        # Pause before the next iteration
        elapsed = time.time() - start_time
        sleep_time = interval - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

if __name__ == "__main__":
    main()
