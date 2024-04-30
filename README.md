# Mini Project 2 - GUI Based Torque Based Control of Kuka IIWA-14 ARM

This Project contains a Graphical User Interface (GUI) that displays and provides controls to the end user for given the robotic arm. 
It also plots the states (joint position, joint velocity, joint acceleration and torques) of the robot in real time.

## Table of Contents

- [Installation and Usage](#InstallationandUsage)
- [Results](#results)

## Installation and Usage

Unzip contents of this project in a new folder along with [Peter Corke's Robotics Toolbox](https://petercorke.com/toolboxes/robotics-toolbox/) for MATLAB (Version 10.4), and follow the steps below:

1. Launch MATLAB and navigate to the project directory.
2. Open the `run_gui.m` script.
3. In the GUI window, you will find input fields where you can specify the desired position of the end effector of the arm in terms of XYZ and RPY coordinates along with payload.
4. Enter the desired values in the input fields.
5. Click on the "Move" button to initiate the calculations.
6. The calculations will start, and the states of the robot, including joint position, joint velocity, joint acceleration, and torques, will be displayed in real-time on the right side of the GUI.

Note: Make sure you have the necessary dependencies and MATLAB toolboxes installed before running the project.

## Results

