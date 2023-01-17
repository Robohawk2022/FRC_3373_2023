// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve.old;


import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.swerve.SwerveConfig;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class MacDrive {

  public static final double POS_CONVERSION = 360.0 / 2048.0;

  public static final double MAGIC_ROTATE_ANGLE = 2.72;

  public static double MaxSpeed = 0.3;
  public static double MaxRotation = 5;
  public static double RotationLimit = 3;
  public static double StrafeLimit = .25;

  private final TalonFX frontLeftDrive;
  private final TalonFX frontRightDrive;
  private final TalonFX backLeftDrive;
  private final TalonFX backRightDrive;
  private final List<TalonFX> driveMotors;

  private final TalonFX frontLeftTurn;
  private final TalonFX frontRightTurn;
  private final TalonFX backLeftTurn;
  private final TalonFX backRightTurn;
  private final List<TalonFX> turnMotors;

  private final XboxController controller;
  private double turboFactor;
  private double reverseFactor;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

/* ==============================================================================
  _____   ____  ____   ____ _______ 
 |  __ \ / __ \|  _ \ / __ \__   __|
 | |__) | |  | | |_) | |  | | | |   
 |  _  /| |  | |  _ <| |  | | | |   
 | | \ \| |__| | |_) | |__| | | |   
 |_|  \_\\____/|____/ \____/  |_|   
                                                                      
============================================================================== */

  public MacDrive(XboxController controller) {

    SmartDashboard.putString("MotorTesting", "None");

    this.controller = controller;

    frontLeftDrive = new TalonFX(SwerveConfig.FL_DRIVE_CANID);
    frontRightDrive = new TalonFX(SwerveConfig.FR_DRIVE_CANID);
    backLeftDrive = new TalonFX(SwerveConfig.BL_DRIVE_CANID);
    backRightDrive = new TalonFX(SwerveConfig.BR_DRIVE_CANID);

    driveMotors = Arrays.asList(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
    for (int i=0; i<driveMotors.size(); i++) {
      driveMotors.get(i).configOpenloopRamp(0.3);
      driveMotors.get(i).setNeutralMode(NeutralMode.Brake);
    }

    frontLeftTurn = new TalonFX(SwerveConfig.FL_TURN_CANID);
    frontRightTurn = new TalonFX(SwerveConfig.FR_TURN_CANID);
    backLeftTurn = new TalonFX(SwerveConfig.BL_TURN_CANID);
    backRightTurn = new TalonFX(SwerveConfig.BR_TURN_CANID);

    turnMotors = Arrays.asList(frontLeftTurn, frontRightTurn, backLeftTurn, backRightTurn);
    for (int i=0; i<turnMotors.size(); i++) {
      turnMotors.get(i).configOpenloopRamp(SwerveConfig.DEFAULT_TURN_RAMP_RATE);
      turnMotors.get(i).setNeutralMode(SwerveConfig.DEFAULT_TURN_NEUTRAL_MODE);
      SwerveConfig.TURN_PID.configPID(turnMotors.get(i));
    }

    turboFactor = 1.0;
    reverseFactor = 1.0;
  }

  public void updateStats() {
    SmartDashboard.putNumber("FL Angle Position", frontLeftTurn.getSelectedSensorPosition() * POS_CONVERSION);
    SmartDashboard.putNumber("FR Angle Position", frontRightTurn.getSelectedSensorPosition() * POS_CONVERSION);
    SmartDashboard.putNumber("BR Angle Position", backRightTurn.getSelectedSensorPosition() * POS_CONVERSION);
    SmartDashboard.putNumber("BL Angle Position", backLeftTurn.getSelectedSensorPosition() * POS_CONVERSION);
    SmartDashboard.putNumber("FL Drive Output", frontLeftDrive.getMotorOutputPercent());
    SmartDashboard.putNumber("FR Drive Output", frontRightDrive.getMotorOutputPercent());
    SmartDashboard.putNumber("BL Drive Output", backLeftDrive.getMotorOutputPercent());
    SmartDashboard.putNumber("BR Drive Output", backRightDrive.getMotorOutputPercent());
    SmartDashboard.putBoolean("Drive Reversed?", reverseFactor < -1.0);
    SmartDashboard.updateValues();
  }

  public void stop() {
    for (int i=0; i<driveMotors.size(); i++) {
      driveMotors.get(i).set(ControlMode.PercentOutput, 0.0);
    }
  }

  public void updateWheels() {

    double rightX = controller.getRightX();
    double leftX = controller.getLeftX();
    double leftY = controller.getLeftY();

    // if someone hits start, we'll invert the "front" of the vehicle for driving
    if (controller.getStartButtonPressed()) {
      reverseFactor = reverseFactor * -1.0;
    }

    // if someone is holding the right trigger, we'll double speed
    if (controller.getRightTriggerAxis() > 0.5) {
      turboFactor = 2.0;
    } else {
      turboFactor = 1.0;
    }

    if (controller.getRightBumper()) {
      AimBot(rightX);
      return;
    }

    if (controller.getLeftBumper()) {
      macDrive(leftX, leftY, rightX);
      return;
    }
    
    driveDrive(leftX, leftY, rightX);
  }

  /* --------------------------------------------------
     Mac Drive turns like this (strafing):
         \---\
         |   |
         \---\
    -------------------------------------------------- */


  public void macDrive(double leftX, double leftY, double rightX) {

    double moveSpeed = Math.sqrt(leftX * leftX + leftY * leftY) * MaxSpeed * turboFactor * reverseFactor;
    double turnAngle = leftX * leftX * leftX * MaxRotation * reverseFactor;    

    if (controller.getLeftY() > 0) {
      setDriveMotorsRightInverted(moveSpeed);
      setTurnMotorsEqual(turnAngle);
    }
    if (controller.getLeftY() < 0) {
      setDriveMotorsRightInverted(-moveSpeed);
      setTurnMotorsEqual(turnAngle);
    }
  }

  /* --------------------------------------------------
     Drive Drive turns like this (car steering):
         /---/
         |   |
         \---\
    -------------------------------------------------- */

  public void driveDrive(double leftX, double leftY, double rightX) {

    double moveSpeed = Math.sqrt(leftX * leftX + leftY * leftY) * MaxSpeed * turboFactor * reverseFactor;
    double turnAngle = rightX * rightX * rightX * MaxRotation * reverseFactor;   
    if (turnAngle > RotationLimit) {
      turnAngle = RotationLimit;
    }

    if (controller.getLeftY() > 0) {
      setDriveMotorsRightInverted(moveSpeed);
      setTurnMotorsEqual(turnAngle);
    }
    if (controller.getLeftY() < 0) {
      setDriveMotorsRightInverted(-moveSpeed);
      setTurnMotors(-turnAngle, -turnAngle, turnAngle, turnAngle);
    }
  }

  /* --------------------------------------------------
     AimBot turns like this (rotation):
         /---\
         |   |
         \---/
    -------------------------------------------------- */

  public void AimBot(double rightX) {
    double rotateSpeed = -rightX / 8.0 * turboFactor;
    setDriveMotors(rotateSpeed, rotateSpeed, rotateSpeed, rotateSpeed);
    setTurnMotors(-MAGIC_ROTATE_ANGLE, MAGIC_ROTATE_ANGLE, -MAGIC_ROTATE_ANGLE, MAGIC_ROTATE_ANGLE);
  }

/* ==============================================================================
  _______ ______  _____ _______ 
 |__   __|  ____|/ ____|__   __|
    | |  | |__  | (___    | |   
    | |  |  __|  \___ \   | |   
    | |  | |____ ____) |  | |   
    |_|  |______|_____/   |_|   
                                                            
============================================================================== */

  public void startTest() {
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);
  }

  public void testPeriodic() {

    SmartDashboard.putString("MotorTesting: ", "None");

    if(controller.getBButton() == true) {
      for (int i=0; i<turnMotors.size(); i++) {
        turnMotors.get(i).setSelectedSensorPosition(0.0);
      }
      System.out.print("Encoders reset");
    }

    double pg = SmartDashboard.getNumber("P Gain", 0);
    double ig = SmartDashboard.getNumber("I Gain", 0);
    double dg = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    boolean changed = false;

    if (pg != kP) { kP = pg; changed = true; }
    if (ig != kI) { kI = ig; changed = true; }
    if (dg != kD) { kD = dg; changed = true; }
    if (iz != kIz) { kIz = iz; changed = true; }
    if (ff != kFF) { kFF = ff; changed = true; }
    if (max != kMaxOutput) { kMaxOutput = max; changed = true; }
    if (min != kMinOutput) { kMinOutput = min; changed = true; }

    if (changed) {
      for (int i=0; i<turnMotors.size(); i++) {
        turnMotors.get(i).config_kP(0, kP);
        turnMotors.get(i).config_kI(0, kI);
        turnMotors.get(i).config_kD(0, kD);
        turnMotors.get(i).config_IntegralZone(0, kIz);
        turnMotors.get(i).config_kF(0, kFF);
        turnMotors.get(i).configPeakOutputForward(max);
        turnMotors.get(i).configPeakOutputReverse(min);
      }
    }

    while(controller.getAButton() == true) {
      SmartDashboard.putString("MotorTesting", "Front Left");
      frontLeftDrive.set(ControlMode.PercentOutput, controller.getLeftY());
      frontLeftTurn.set(ControlMode.PercentOutput, controller.getRightY());
    }
    while(controller.getBButton() == true) {
      SmartDashboard.putString("MotorTesting", "Front Right");
      frontRightDrive.set(ControlMode.PercentOutput, controller.getLeftY());
      frontRightTurn.set(ControlMode.PercentOutput, controller.getRightY());
    }
    while(controller.getXButton() == true) {   
       SmartDashboard.putString("MotorTesting", "Back Left");
      backLeftDrive.set(ControlMode.PercentOutput, controller.getLeftY());
      backLeftTurn.set(ControlMode.PercentOutput, controller.getRightY());
    }
    while(controller.getYButton() == true) {
      SmartDashboard.putString("MotorTesting", "Back Right");
      backRightDrive.set(ControlMode.PercentOutput, controller.getLeftY());
      backRightTurn.set(ControlMode.PercentOutput, controller.getRightY());
    }
  }

  public void setDriveMotorsRightInverted(double val) {
    setDriveMotors(val, -val, -val, val);
  }

  public void setDriveMotors(double fl, double fr, double br, double bl) {
    frontLeftDrive.set(ControlMode.PercentOutput, fl);
    frontRightDrive.set(ControlMode.PercentOutput, fr);
    backRightDrive.set(ControlMode.PercentOutput, br);
    backLeftDrive.set(ControlMode.PercentOutput, bl);
  }

  public void setTurnMotorsEqual(double val) {
    setTurnMotors(val, val, val, val);
  }

  public void setTurnMotors(double fl, double fr, double br, double bl) {
    frontLeftTurn.set(ControlMode.Position, fl);
    frontRightTurn.set(ControlMode.Position, fr);
    backRightTurn.set(ControlMode.Position, br);
    backLeftTurn.set(ControlMode.Position, bl);
  }
}