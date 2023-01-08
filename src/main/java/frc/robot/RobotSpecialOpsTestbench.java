// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.motors.MotorFactory;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Implementation of a Robot that only has the shooting subsystem, so we can develop
 * and test independently.
 */
public class RobotSpecialOpsTestbench extends TimedRobot {
  
  public static final int CONTROLLER_PORT = 0;
  public static final int INTAKE_PORT = 1;
  public static final int SHOOTER_LAUNCH_PORT = 4;
  public static final int SHOOTER_INDEXER_PORT = 3;
  public static final int SHOOTER_SWITCH_PORT = 3;

  public static final boolean USE_CAMERAS = false; // set to true to enable cameras
  public static final int FRONT_CAMERA_PORT = 0;
  public static final int BACK_CAMERA_PORT = 1;

  private XboxController controller;
  private ShooterSubsystem shooter;
  private IntakeSubsystem intake;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    System.err.println("initializing robot ...");
    controller = new XboxController(CONTROLLER_PORT);
    intake = new IntakeSubsystem(controller, INTAKE_PORT);
    shooter = new ShooterSubsystem(controller, SHOOTER_LAUNCH_PORT, SHOOTER_INDEXER_PORT, SHOOTER_SWITCH_PORT);
    if (!isSimulation() && USE_CAMERAS) {
      CameraServer.startAutomaticCapture("Front", FRONT_CAMERA_PORT);
      CameraServer.startAutomaticCapture("Back", BACK_CAMERA_PORT);  
    }
  }

  /** This function is called periodically in all modes */
  @Override
  public void robotPeriodic() {
    shooter.robotPeriodic();
    intake.robotPeriodic();
    MotorFactory.updateDashboard();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    shooter.teleopPeriodic();
    intake.telopPeriodic();
  }
  
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    shooter.disabledInit();
    intake.disabledInit();
  }
}