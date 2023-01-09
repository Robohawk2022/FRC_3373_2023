# Team Robohawk 2023

Code for our FRC competition bot for the 2023 season ("Charged Up").


# Design notes

## Swerve drive

- TBD

## Arm

- TBD

## Hand

- TBD

## Vision

- TBD


# TODO

- Create skeletons for subsystems
    - Drive
    - Arm
    - Hand
    - Vision?

- Image a new SD card for the new RoboRIO 2.0

- Try out the Phoenix Tuner with a Falcon motor

- Get a Falcon motor working on the test bench


# Software you may want to install

## Editing code

To write robot code, you'll want to use Visual Studio Code. Information including installation instructions can be found here:

    https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html

_NOTE: You will want this version of Visual Studio Code - it has custom plugins that do all the important tasks for building and deploying robot code. Even if you installed a copy for the 2022 season, you should install a fresh one for this new season (they've totally rewritten the plugins)._

## Setting up a project

If you want to set up a new project for any reason, you can do it using the custom action ```WPILib: Create new product``` in VS Code. The vendor libraries we use can be installed using the ```WPILib: Manage Vendor Libraries``` custom action.

The NEO motors (with the Spark Controllers) are supplied by REV Robotics. The link to their software is here:

    https://software-metadata.revrobotics.com/REVLib-2023.json

The Falcon motors (with built-in Talon controllers) come from CTRE. This is their link:

    https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix5-frc2023-latest.json

## FRC game utilities

To deploy to and control software on the RoboRIO, you'll need the FRC Driver Station. Instructions for deploying it are here:

    https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/frc-game-tools.html

You might get asked for a license - we're working on getting that squared away but, until then, the license should be for some optional parts and you don't need them.

## Hardware utilities

REV Robotics has a "hardware client" that lets you interface a PC directly with the Spark motor controllers (without the need for the RoboRIO or writing any Java code). You can download that from here:

    https://docs.revrobotics.com/rev-hardware-client/

CTRE provides a whole mess of software (programming libraries and other stuff) under the name "Phoenix". It looks like the equivalent to the REV client is the "Phoenix Tuner". Instructions for finding and downloading "Phoenix Tuner X" are here:

    https://store.ctr-electronics.com/software/
