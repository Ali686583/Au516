# Sending a Python Script to the Raspberry Pi 5 Using SCP
This tutorial explains how to transfer a file from your laptop to your robot’s Raspberry Pi 5 using the scp command.

## 1. Make sure you know the robot’s IP address </br>
For Robot 2, the Raspberry Pi 5 has the IP:

```Code
192.168.2.1
```
The default username is:

```Code
pi
```

## 2. Open a terminal on your laptop
- On Windows, use PowerShell
- On macOS/Linux, use your standard terminal

Navigate to the folder containing the Python script you want to send.

Example:
```bash
cd path/to/your/script/
```

## 3. Use SCP to send the file to the robot </br>
To copy **scan_viewer.py** to the robot’s ROS2 package directory:

```bash
scp ./scan_viewer.py pi@192.168.2.1:/home/pi/ros2_ws/src/AU516/au516_nav/au516_nav/
```

Explanation:
- scp → secure copy
- ./scan_viewer.py → file on your laptop
- pi@192.168.2.1 → username + robot IP
- /home/pi/... → destination folder on the robot

## 4. Enter the password
When prompted, type the Raspberry Pi password (provided by the teacher).

## 5. Verify on the robot
Access the RPi5 content with SSH:
```bash
ssh pi@192.168.2.1
```

Then check the folder:
```bash
ls /home/pi/ros2_ws/src/AU516/au516_nav/au516_nav/
```

You should see:
```Code
scan_viewer.py
```

Display the content of the file:
```bash
cat /home/pi/ros2_ws/src/AU516/au516_nav/au516_nav/scan_viewer.py
```
You should see the modified content of your file on the RPi5.