# Robohawk 2023 Lab TODO

----

## Chassis configuration

Use the Phoenix tuner to do all this.

Assign CAN IDs
  * Make sure to note which is front left, front right, etc.
  * We start at 10, and each wheel has 3 assignments:
    * Drive motors should be 10, 13, 16, 19 
    * Turn motors should be 11, 14, 17, 20 
    * Turn encoders should be 12, 15, 18, 21
  
For each wheel
  * Update the firmware on both motors
  * Use the Tuner to spin both motors, to make sure they work

## Position PID tuning

Our turn motors will be doing [position-based closed loop control](https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#position-closed-loop-control-mode).
(If you read the CTRE doc - we are NOT doing "auxiliary control".)

Update `Main.java` to point to the `RobotTalonPositionPIDTest` and deploy it, but leave it
enabled in autonomous mode. Do this testing using the Phoenix Tuner, like we did for 
velocity PID.

It looks like there's a [150/7:1 gear ratio](https://www.andymark.com/products/mk4i-swerve-modules)
for the turn motor. This means a 360 rotation of the wheel will require ~26 revolutions of
the turn motor shaft.

Tuning process ...

FIRST get the robot deployed and the Tuner connected, and check the PID parameters.  They're
on the Config Tab under "Slot 0"; you should use the defaults from the Java code as a starting
point.

NEXT use the "Plot tab" to show you the motor's position value.  Note what it's reporting,
then rotate the wheel around one full rotation by hand. See what the new value is. [It looks
like](https://docs.ctre-phoenix.com/en/stable/ch12a_BringUpCANCoder.html) the CANcoder reports
the rotation of the motor shaft in degrees, so one wheel rotation should change the motor's
drive shaft by about ~26 x 360 degrees.

NEXT ... use the Control tab to set a new position value, and make the wheel spin to meet
it. Try moving the wheel through 45, 90, and 180 degree turns to make sure you can send it
accurately to a particular position. The wheel should stop turning once it hits the target
position - if you try to move it by hand, it should move back.

If the wheel "undershoots", you probably need to adjust the P value upward. If it oscillates
around the target point, you should probably adjust I upward. Use the Tuner to adjust the
values and run your tests again.

Once you have a set of PID values that you like, PUT THEM BACK INTO THE CODE, and push it
to GitHub so we keep it.

## NavX

This is the gyro that tells us our heading and pitch.

We have several generations of them - the ones we want to use this year are in unopened
bags inside black boxes, in the box underneath the test bench. They will plug into the
RoboRIO using a USB port.

`RobotNavxOnly` is, as the name implies, a testbot for the NavX. While it's
running, it will print out the following values from the NavX:

* Pitch (tip the card back and forth to see how this works)
* Heading (rotate the card around see how this works)
* Displacement in X and Y (move the card around to see how this works)

For the swerve drive, we need to make sure we can calculate the
robot's heading in degrees from 0 to 360. Play around with the card
and the robot code to see how this works.

# Software install links

----

## FRC / WPI Software

* Important stuff
  * [VS Code](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html)
  * [FRC game tools, including the Driver Station](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html)

* Optional stuff
  * [RoboRIO imaging tools](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-3/imaging-your-roborio.html)
  * [RoboRIO radio configuration](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-3/radio-programming.html)

If you are asked to create an NI account for a license, just skip it or hit "Extend Trial License"

# Vendor Software

* REV Robotics - NEO motors, Spark controllers
  * [WPI Lib vendor library URL](https://software-metadata.revrobotics.com/REVLib-2023.json)
  * [2023 Javadoc](https://codedocs.revrobotics.com/java/com/revrobotics/package-summary.html)
  * [Sample code](https://github.com/REVrobotics/SPARK-MAX-Examples/tree/master/Java)
  * [REV hardware client](https://docs.revrobotics.com/rev-hardware-client/)

* CTRE Phoenix - Falcon motors, TalonFX controllers and CANcoder
  * [WPI Lib vendor library URL](https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix5-frc2023-latest.json)
  * [2023 Javadoc](https://api.ctr-electronics.com/phoenix/release/java/)
  * [Full user guide](https://v5.docs.ctr-electronics.com/en/stable/)
  * [Sample code](https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/tree/master/Java%20Talon%20FX%20(Falcon%20500))
  * [Phoenix Tuner X](https://pro.docs.ctr-electronics.com/en/stable/docs/tuner/index.html)

* Kauai Labs - NavX
  * [WPI Lib vendor library URL](https://www.kauailabs.com/dist/frc/2022/navx_frc.json)
  * [Install and code samples](https://pdocs.kauailabs.com/navx-micro/examples/)
