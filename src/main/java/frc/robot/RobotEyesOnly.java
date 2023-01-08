// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.motors.MotorFactory;
import frc.robot.subsystems.ClimberSubsystem;

/**
 * Implementation of a Robot that only has the climber subsystem, so we can develop
 * and test independently.
 */
public class RobotEyesOnly extends TimedRobot {

  public static final int FRONT_CAMERA_PORT = 0;
  public static final int BACK_CAMERA_PORT = 1;
  public static final int BALL_CAMERA_PORT = 2;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    if (!isSimulation()) {
      CameraServer.startAutomaticCapture("Front", FRONT_CAMERA_PORT);
      CameraServer.startAutomaticCapture("Back", BACK_CAMERA_PORT);  
      CameraServer.startAutomaticCapture("Ball",BALL_CAMERA_PORT);
    }
  }

  /** This function is called periodically in all modes */
  @Override
  public void robotPeriodic() {
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
  }
  
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    
  }
}