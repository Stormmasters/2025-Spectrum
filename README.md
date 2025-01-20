# Spectrum Robot Code

Code structure for our 2025 FRC Robot Code

## Description

This is the working code base for our beta testing and development in the fall of 2024 leading into 2025. This will likely be used for some prototyping in 2025 as well. Before we fork to a new repo for the 2025 competition code.

### Structure

* Robot = In season robot code, we have configuration files to be able to run our code base on multiple robots at once. Heavily uses WPILib commands and triggers.
* SpectrumLib = Code that we try to reuse year to year.
* Our goal is to make our software easy for multiple people to contribute too. Each subsystem should be able to be modified on it's own without needing to understand the rest of the robot code.

### Features

* Mechanism class wraps all the FalconFX configurations and control mode calls so they are easier to use.
* Heavy use of Triggers as states for activating commands. This all but eliminates the need for complex multi-mechanism command groups.
* Simulation classes allow us to build a good representation of the complete robot in sim so more of the code can be tested without the robot.
* CachedDouble: Allows a value to only be updated once per periodic loop. This is useful for CANbus calls to sensors or motors, etc.
* LED classes that use the WPILib Addressable LED for animations, easily have multiple strips on one port, has priorities for when to override animations that are currently running, different animations for each control mode, etc.

### Build Tools and Extensions

* **Spotless:** Autoformats are code on each build so that we keep a consistent format across programmers.
* **SpotBugs**: Catches some common bugs in the software such as using = instead of == in a conditional, etc.
* **Lomok:** Annotations to create Getter and Setter methods. This allows for less boilerplate code.
* **Error Lens:** Highlights errors and makes them easier to see and fix.
* **Git Config User Profiles:** Allows multiple programmers to share the same computers and commit under their own names.
* **Git Lens:** lets you see who committed changes and more
* **SpellRight:** Spell Check for VSCode

### Dependencies

* WPILib
* CTRE Phoenix 6 (Using there swerve control code)
* PathPlanner
* PhotonLib (For vision simulation)
* GrappleLib (when 2025 is released)
