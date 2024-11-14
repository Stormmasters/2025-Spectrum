# Spectrum Robot Code

Code structure for our 2025 FRC Robot Code

## Description

This is the working code base for our beta testing and development in the fall of 2024 leading into 2025. This will likely be used for some prototyping in 2025 as well. Before we fork to a new repo for the 2025 competition code.

### Dependencies

* WPILib
* CTRE Phoenix 6 (Using there swerve control code)
* PathPlanner
* PhotonLib (For vision simulation)
* GrappleLib (when 2025 is released)

### Structure

* Robot = In season robot code, we have configuration files to be able to run our code base on multiple robots at once. Heavily uses WPIib commands and triggers.
* SpectrumLib = Code that we try to reuse year to year.
* Our goal is to make our software easy for multiple people to contribute too.
