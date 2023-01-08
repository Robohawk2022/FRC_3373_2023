// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.motors.MotorFactory;
import frc.robot.subsystems.ClimberSubsystem;

/**
 * Implementation of a Robot that only has the climber subsystem, so we can develop
 * and test independently.
 */
public class RobotClimberOnly extends TimedRobot {
  
  public static final int CONTROLLER_PORT = 0;
  public static final int EXTENDER_PORT = 1;
  public static final int ROTATOR_PORT = 4;
  public static final int EXTENDER_SWITCH = 0;
  public static final int ROTATOR_SWITCH = 3;

  public static final boolean USE_CAMERAS = false;
  public static final int FRONT_CAMERA_PORT = 0;
  public static final int BACK_CAMERA_PORT = 1;

  private XboxController specialOpsController;
  private ClimberSubsystem climber;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    specialOpsController = new XboxController(CONTROLLER_PORT);
    climber = new ClimberSubsystem(specialOpsController, EXTENDER_PORT, EXTENDER_SWITCH, ROTATOR_PORT, ROTATOR_SWITCH, null);
  }

  /** This function is called periodically in all modes */
  @Override
  public void robotPeriodic() {
    climber.robotPeriodic();
    MotorFactory.updateDashboard();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    climber.teleopPeriodic();
  }
  
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    climber.disabledInit();
  }
}