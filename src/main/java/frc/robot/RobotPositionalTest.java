// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.motors.MotorFactory;
import frc.robot.motors.NamedMotor;
import frc.robot.motors.PositionClosedLoopMotor;
import frc.robot.util.Logger;

/**
 * Implementation of a Robot for testing positional motor
 */
public class RobotPositionalTest extends TimedRobot {
  
  public static final int CONTROLLER_PORT = 0;
  public static final int MOTOR_PORT = 3;
  private static final double MAX_SPEED = 0.5;
  private static final double THRESH = 0.01;

  private XboxController controller;
  private PositionClosedLoopMotor motor;
  private boolean motorEnabled;
  private double targetPosition;
  private double totalDelta;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    controller = new XboxController(CONTROLLER_PORT);
    motor = new PositionClosedLoopMotor("Motor", MOTOR_PORT);
  }

  /** This function is called periodically in all modes */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putBoolean("Motor Enabled?", motorEnabled);
    SmartDashboard.putNumber("Target Position", targetPosition);
    MotorFactory.updateDashboard();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    motorEnabled = false;
    motor.set(0.0);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    // the back button toggles the motor's enabled status
    if (controller.getBackButtonPressed()) {
      motorEnabled = !motorEnabled;
      if (motorEnabled) {
        motor.resetClosedLoopControl();
      }
    }

    // if the motor is enabled, one of three things happens:
    //    - if someone presses the X button, we'll move the motor backward 100
    //    - if someone presses the Y button, we'll move the motor forward 100
    //    - otherwise, we'll hold still at the current position
    if (motorEnabled) {

      if (controller.getXButtonPressed()) {
        Logger.log("decreasting target position to "+targetPosition);
        motor.rotate(-20);
      } else if (controller.getYButtonPressed()) {
        Logger.log("increasing target position to "+targetPosition);
        motor.rotate(20);
      }
      motor.updateSpeed();
    }

    // if the motor is not enabled, turn off power in brake mode
    else {
      motor.set(0);
    }
  }
}