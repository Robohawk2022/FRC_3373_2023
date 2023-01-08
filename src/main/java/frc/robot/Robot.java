// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.motors.MotorFactory;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.Logger;
import edu.wpi.first.wpilibj.Timer;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public static final int DRIVER_PORT = 0;
  public static final int SPECIAL_OPS_PORT = 1;
  public static final int INTAKE_PORT = 9;
  public static final int SHOOTER_LAUNCH_PORT = 10;
  public static final int SHOOTER_INDEXER_PORT = 11;
  public static final int SHOOTER_SWITCH_PORT = 1;
  public static final int CLIMBER_EXTENDER_PORT = 12;
  public static final int CLIMBER_ROTATOR_PORT = 13;
  public static final int CLIMBER_EXTENDER_SWITCH = 3;
  public static final int CLIMBER_ROTATOR_SWITCH = 4;
  public static final int FRONT_LEFT_ANGLE_ID = 8;
  public static final int FRONT_LEFT_DRIVE_ID = 7;
  public static final int FRONT_RIGHT_ANGLE_ID = 6;
  public static final int FRONT_RIGHT_DRIVE_ID = 5;
  public static final int BACK_RIGHT_ANGLE_ID = 4;
  public static final int BACK_RIGHT_DRIVE_ID = 3;
  public static final int BACK_LEFT_ANGLE_ID = 2;
  public static final int BACK_LEFT_DRIVE_ID = 1;

  public static final boolean USE_CAMERAS = true;
  public static final int FRONT_CAMERA_PORT = 0;
  public static final int BACK_CAMERA_PORT = 1;

  public static double MaxSpeed = 0.3;
  public static double MaxRotation = 5;
  public static double RotationLimit = 3;
  public static double StrafeLimit = .25;
  public static double MagicRotateAngle = 2.72;

  private XboxController specialops;
  private IntakeSubsystem intake;
  private ShooterSubsystem shooter;
  private ClimberSubsystem climber;
  private CANSparkMax frontLeftAngleMotor;
  private CANSparkMax frontLeftDriveMotor;
  private CANSparkMax frontRightDriveMotor;
  private CANSparkMax frontRightAngleMotor;
  private CANSparkMax backLeftAngleMotor;
  private CANSparkMax backLeftDriveMotor;
  private CANSparkMax backRightDriveMotor;
  private CANSparkMax backRightAngleMotor;
  private SparkMaxPIDController frontLeftPidController;
  private SparkMaxPIDController frontRightPidController;
  private SparkMaxPIDController backRightPidController;
  private SparkMaxPIDController backLeftPidController;
  private RelativeEncoder frontLeftAngleEncoder;
  private RelativeEncoder frontRightAngleEncoder;
  private RelativeEncoder backRightAngleEncoder;
  private RelativeEncoder backLeftAngleEncoder;
  private XboxController drive_control;
  private double turboFactor;
  private double reverseFactor;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private double autonomousStart;
  private SendableChooser<String> autoMode;
  private long teleopRounds;

/* ==============================================================================
  _____   ____  ____   ____ _______ 
 |  __ \ / __ \|  _ \ / __ \__   __|
 | |__) | |  | | |_) | |  | | | |   
 |  _  /| |  | |  _ <| |  | | | |   
 | | \ \| |__| | |_) | |__| | | |   
 |_|  \_\\____/|____/ \____/  |_|   
                                                                      
============================================================================== */

  @Override
  public void robotInit() {
    SmartDashboard.putString("MotorTesting: ", "None");
    drive_control = new XboxController(DRIVER_PORT);
    intake = new IntakeSubsystem(drive_control, INTAKE_PORT);
    
    specialops = new XboxController(SPECIAL_OPS_PORT);
    shooter = new ShooterSubsystem(specialops, SHOOTER_LAUNCH_PORT, SHOOTER_INDEXER_PORT, SHOOTER_SWITCH_PORT);
    climber = new ClimberSubsystem(specialops, CLIMBER_EXTENDER_PORT, CLIMBER_EXTENDER_SWITCH, CLIMBER_ROTATOR_PORT, CLIMBER_ROTATOR_SWITCH, () -> {
      shooter.disabledInit();
      intake.disabledInit();
    });

    frontLeftAngleMotor = new CANSparkMax(FRONT_LEFT_ANGLE_ID, MotorType.kBrushed);
    frontLeftDriveMotor = new CANSparkMax(FRONT_LEFT_DRIVE_ID, MotorType.kBrushless);
    frontRightAngleMotor = new CANSparkMax(FRONT_RIGHT_ANGLE_ID, MotorType.kBrushed);
    frontRightDriveMotor = new CANSparkMax(FRONT_RIGHT_DRIVE_ID, MotorType.kBrushless);
    backRightAngleMotor = new CANSparkMax(BACK_RIGHT_ANGLE_ID, MotorType.kBrushed);
    backRightDriveMotor = new CANSparkMax(BACK_RIGHT_DRIVE_ID, MotorType.kBrushless);
    backLeftDriveMotor = new CANSparkMax(BACK_LEFT_DRIVE_ID, MotorType.kBrushless);
    backLeftAngleMotor = new CANSparkMax(BACK_LEFT_ANGLE_ID, MotorType.kBrushed);

    frontLeftPidController = frontLeftAngleMotor.getPIDController();
    frontRightPidController = frontRightAngleMotor.getPIDController();
    backRightPidController = backRightAngleMotor.getPIDController();
    backLeftPidController = backLeftAngleMotor.getPIDController();

    // RESET SPARK MAX
    frontLeftAngleMotor.restoreFactoryDefaults();
    frontRightAngleMotor.restoreFactoryDefaults();
    frontLeftDriveMotor.restoreFactoryDefaults();
    frontRightDriveMotor.restoreFactoryDefaults();
    backLeftAngleMotor.restoreFactoryDefaults();
    backRightAngleMotor.restoreFactoryDefaults();
    backLeftDriveMotor.restoreFactoryDefaults();
    backRightDriveMotor.restoreFactoryDefaults();

    frontRightDriveMotor.setOpenLoopRampRate(1.0);
    frontLeftDriveMotor.setOpenLoopRampRate(1.0);
    backRightDriveMotor.setOpenLoopRampRate(1.0);
    backLeftDriveMotor.setOpenLoopRampRate(1.0);

    autoMode = new SendableChooser<>();
    autoMode.setDefaultOption("SingleShooter", "SingleShooter");
    autoMode.addOption("DoubleShooter", "DoubleShooter");
    autoMode.addOption("ClimberOnly", "ClimberOnly");
    autoMode.addOption("IntakeOnly", "IntakeOnly");
    SmartDashboard.putData("Auto Mode", autoMode);

    frontLeftAngleEncoder = frontLeftAngleMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 104);
    frontLeftAngleEncoder.setPosition(0);
    frontLeftAngleEncoder.setInverted(false);

    frontRightAngleEncoder = frontRightAngleMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 104);
    frontRightAngleEncoder.setInverted(false);
    frontRightAngleEncoder.setPosition(0);

    backRightAngleEncoder = backRightAngleMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 104);
    backRightAngleEncoder.setInverted(false);
    backRightAngleEncoder.setPosition(0);

    backLeftAngleEncoder = backLeftAngleMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 104);
    backLeftAngleEncoder.setInverted(false);
    backLeftAngleEncoder.setPosition(0);

    turboFactor = 1.0;
    reverseFactor = 1.0;

    // kP = 75; 
    // kI = 1e-3;
    // kD = 1; 
    // kIz = 0; 
    // kFF = 0; 
    // kMaxOutput = 1; 
    // kMinOutput = -1;

    kP = 1; 
    kI = 1e-4;
    kD = 1; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    frontLeftPidController.setFeedbackDevice(frontLeftAngleEncoder);
    frontRightPidController.setFeedbackDevice(frontRightAngleEncoder);
    backRightPidController.setFeedbackDevice(backRightAngleEncoder);
    backLeftPidController.setFeedbackDevice(backLeftAngleEncoder);

    frontLeftPidController.setP(kP);
    frontRightPidController.setP(kP);
    backRightPidController.setP(kP);
    backLeftPidController.setP(kP);

    frontLeftPidController.setI(kI);
    frontRightPidController.setI(kI);
    backRightPidController.setI(kI);
    backLeftPidController.setI(kI);

    frontLeftPidController.setD(kD);
    frontRightPidController.setD(kD);
    backRightPidController.setD(kD);
    backLeftPidController.setD(kD);

    frontLeftPidController.setIZone(kIz);
    frontRightPidController.setIZone(kIz);
    backRightPidController.setIZone(kIz);
    backLeftPidController.setIZone(kIz);

    frontLeftPidController.setFF(kFF);
    frontRightPidController.setFF(kFF);
    backRightPidController.setFF(kFF);
    backLeftPidController.setFF(kFF);

    frontLeftPidController.setOutputRange(kMinOutput, kMaxOutput);
    frontRightPidController.setOutputRange(kMinOutput, kMaxOutput);
    backRightPidController.setOutputRange(kMinOutput, kMaxOutput);
    backLeftPidController.setOutputRange(kMinOutput, kMaxOutput);

    
    frontLeftAngleMotor.setInverted(true);
    frontRightAngleMotor.setInverted(true);
    backLeftAngleMotor.setInverted(true);
    backRightAngleMotor.setInverted(true);

    if (USE_CAMERAS && !isSimulation()) {
      CameraServer.startAutomaticCapture("Front", FRONT_CAMERA_PORT);
      CameraServer.startAutomaticCapture("Back", BACK_CAMERA_PORT);  
    }

    autonomousStart = 0.0;
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    if (intake != null) {
      intake.robotPeriodic();
    }
    if (shooter != null) {
      shooter.robotPeriodic();
    }
    if (climber != null) {
      climber.robotPeriodic();
    }
    SmartDashboard.putNumber("FL Angle Position", frontLeftAngleEncoder.getPosition());
    SmartDashboard.putNumber("FR Angle Position", frontRightAngleEncoder.getPosition());
    SmartDashboard.putNumber("BR Angle Position", backRightAngleEncoder.getPosition());
    SmartDashboard.putNumber("BL Angle Position", backLeftAngleEncoder.getPosition());
    SmartDashboard.putBoolean("Drive Reversed?", reverseFactor < -1.0);
    SmartDashboard.putNumber("Drive Counter", teleopRounds);
  }

/* ==============================================================================
          _    _ _______ ____  _   _  ____  __  __  ____  _    _  _____ 
     /\  | |  | |__   __/ __ \| \ | |/ __ \|  \/  |/ __ \| |  | |/ ____|
    /  \ | |  | |  | | | |  | |  \| | |  | | \  / | |  | | |  | | (___  
   / /\ \| |  | |  | | | |  | | . ` | |  | | |\/| | |  | | |  | |\___ \ 
  / ____ \ |__| |  | | | |__| | |\  | |__| | |  | | |__| | |__| |____) |
 /_/    \_\____/   |_|  \____/|_| \_|\____/|_|  |_|\____/ \____/|_____/ 
                                                                                                                                              
============================================================================== */

  @Override
  public void autonomousInit() {

    // always clock the start of autonomous mode, and run the climber
    autonomousStart = Timer.getFPGATimestamp();
    climber.autonomousInit();

    Logger.log("starting auto program ", autoMode.getSelected());

    if ("DoubleShooter".equalsIgnoreCase(autoMode.getSelected())) {
      intake.doubleShooterInit();
      shooter.doubleShooterInit();
    } else if ("IntakeOnly".equalsIgnoreCase(autoMode.getSelected())) {
      intake.singleShooterInit();
    } else if ("SingleShooter".equalsIgnoreCase(autoMode.getSelected())) {
      intake.singleShooterInit();
      shooter.singleShooterInit();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    // always run the climber
    climber.autonomousPeriodic();

    double seconds = Timer.getFPGATimestamp() - autonomousStart;

    if ("DoubleShooter".equalsIgnoreCase(autoMode.getSelected())) {
      intake.doubleShooterPeriodic(seconds);
      shooter.doubleShooterPeriodic(seconds);
      doubleShooterPeriodic(seconds);
    } else if ("IntakeOnly".equalsIgnoreCase(autoMode.getSelected())) {
      intake.doubleShooterPeriodic(seconds);
    } else if ("SingleShooter".equalsIgnoreCase(autoMode.getSelected())) {
      intake.singleShooterPeriodic(seconds);
      shooter.singleShooterPeriodic(seconds);
      singleShooterPeriodic(seconds);
    }
  }

  private void singleShooterPeriodic(double seconds) {
    if (seconds < 0.2) {   // let the intake wheel wind up a bit
      forwardBy(0.0, 0.0);
    } else if (seconds < 0.4) {   // ooch forward (picks up speed)
      forwardBy(1.0, 0.0);
    } else if (seconds < 0.9) {   // ooch backwards (drops intake frame)
      forwardBy(-1.0, 0.0);
    } else if (seconds < 6.1) {  // wait in place for intake to drop
      forwardBy(0.0, 0.0);
    } else if (seconds < 9.4) {   // exit the tarmac
      forwardBy(0.1, 0.0);
    } else {
      forwardBy(0.0, 0.0);
    }
  }

  private void doubleShooterPeriodic(double seconds) {
    if (seconds < 0.3) {   // let the intake wheel wind up a bit
      forwardBy(0.0, 0.0);
    } else if (seconds < 0.5) {   // ooch forward (picks up speed)
      forwardBy(1.0, 0.0);
    } else if (seconds < 1.0) {   // ooch backwards (drops intake frame)
      forwardBy(-1.0, 0.0);
    } else if (seconds < 6.2) {  // wait in place for intake to drop
      forwardBy(0.0, 0.0);
    } else if (seconds < 9.0) {   // go get a ball
      forwardBy(0.1, 0.0);
    } else if (seconds < 9.3) {   // reorient slightly to shoot
      forwardBy(0.1, 0.0);
    } else {
      forwardBy(0.0, 0.0);
    }
  }

  private void forwardBy(double speed, double angle) {
    frontLeftPidController.setReference(0, CANSparkMax.ControlType.kPosition);
    frontRightPidController.setReference(0, CANSparkMax.ControlType.kPosition);
    backRightPidController.setReference(angle, CANSparkMax.ControlType.kPosition);
    backLeftPidController.setReference(angle, CANSparkMax.ControlType.kPosition);  
    frontLeftDriveMotor.set(-speed);
    frontRightDriveMotor.set(speed);
    backLeftDriveMotor.set(-speed);
    backRightDriveMotor.set(speed);
  }

/* ==============================================================================
  _______ ______ _      ______ ____  _____  
 |__   __|  ____| |    |  ____/ __ \|  __ \ 
    | |  | |__  | |    | |__ | |  | | |__) |
    | |  |  __| | |    |  __|| |  | |  ___/ 
    | |  | |____| |____| |___| |__| | |     
    |_|  |______|______|______\____/|_|     
                                                                                      
============================================================================== */

  @Override
  public void teleopInit() {
    teleopRounds = 0L;
    shooter.teleopInit();
    intake.teleopInit();
    climber.teleopInit();
  }

  @Override
  public void teleopPeriodic() {

    teleopRounds++;

    SmartDashboard.updateValues();
    //LOGGING TO SD ~ NOT NEEDED
    SmartDashboard.putNumber("Front Left Swerve Wheel Output", frontLeftDriveMotor.get());
    SmartDashboard.putNumber("Back Left Swerve Wheel Output", backLeftDriveMotor.get());
    if (intake != null) {
      intake.telopPeriodic();
    }
    if (shooter != null) {
      shooter.teleopPeriodic();      
    }
    if (climber != null) {
      climber.teleopPeriodic();
    }

    double rightX = drive_control.getRightX();
    double leftX = drive_control.getLeftX();
    double leftY = drive_control.getLeftY();

    // if someone hits start, we'll invert the "front" of the vehicle for driving
    if (drive_control.getStartButtonPressed()) {
      reverseFactor = reverseFactor * -1.0;
    }

    // if someone is holding the right trigger, we'll double speed
    if (drive_control.getRightTriggerAxis() > 0.5) {
      turboFactor = 2.0;
    } else {
      turboFactor = 1.0;
    }

    if (drive_control.getRightBumper()) {
      AimBot(rightX);
      return;
    }

    if (drive_control.getLeftBumper()) {
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

    if (drive_control.getLeftY() > 0) {
      frontLeftDriveMotor.set(moveSpeed);
      frontRightDriveMotor.set(-moveSpeed);
      backRightDriveMotor.set(-moveSpeed);
      backLeftDriveMotor.set(moveSpeed);
      frontLeftPidController.setReference(turnAngle, CANSparkMax.ControlType.kPosition);
      frontRightPidController.setReference(turnAngle, CANSparkMax.ControlType.kPosition);
      backRightPidController.setReference(turnAngle, CANSparkMax.ControlType.kPosition);
      backLeftPidController.setReference(turnAngle, CANSparkMax.ControlType.kPosition);            
    }

    if (drive_control.getLeftY() < 0) {
      frontLeftDriveMotor.set(-moveSpeed);
      frontRightDriveMotor.set(moveSpeed);
      backRightDriveMotor.set(moveSpeed);
      backLeftDriveMotor.set(-moveSpeed);
      frontLeftPidController.setReference(turnAngle, CANSparkMax.ControlType.kPosition);
      frontRightPidController.setReference(turnAngle, CANSparkMax.ControlType.kPosition);
      backRightPidController.setReference(turnAngle, CANSparkMax.ControlType.kPosition);
      backLeftPidController.setReference(turnAngle, CANSparkMax.ControlType.kPosition);            
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

    if (drive_control.getLeftY() > 0) {
      frontLeftDriveMotor.set(moveSpeed);
      frontRightDriveMotor.set(-moveSpeed);
      backRightDriveMotor.set(-moveSpeed);
      backLeftDriveMotor.set(moveSpeed);
      frontLeftPidController.setReference(turnAngle, CANSparkMax.ControlType.kPosition);
      frontRightPidController.setReference(turnAngle, CANSparkMax.ControlType.kPosition);
      backRightPidController.setReference(-turnAngle, CANSparkMax.ControlType.kPosition);
      backLeftPidController.setReference(-turnAngle, CANSparkMax.ControlType.kPosition);            
    }

    if (drive_control.getLeftY() < 0) {
      frontLeftDriveMotor.set(-moveSpeed);
      frontRightDriveMotor.set(moveSpeed);
      backRightDriveMotor.set(moveSpeed);
      backLeftDriveMotor.set(-moveSpeed);
      frontLeftPidController.setReference(-turnAngle, CANSparkMax.ControlType.kPosition);
      frontRightPidController.setReference(-turnAngle, CANSparkMax.ControlType.kPosition);
      backRightPidController.setReference(turnAngle, CANSparkMax.ControlType.kPosition);
      backLeftPidController.setReference(turnAngle, CANSparkMax.ControlType.kPosition);            
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
    frontLeftDriveMotor.set(rotateSpeed);
    frontRightDriveMotor.set(rotateSpeed);
    backLeftDriveMotor.set(rotateSpeed);
    backRightDriveMotor.set(rotateSpeed);  
    frontLeftPidController.setReference(-MagicRotateAngle,  CANSparkMax.ControlType.kPosition);
    frontRightPidController.setReference(MagicRotateAngle,  CANSparkMax.ControlType.kPosition);
    backRightPidController.setReference(-MagicRotateAngle,  CANSparkMax.ControlType.kPosition);
    backLeftPidController.setReference(MagicRotateAngle,  CANSparkMax.ControlType.kPosition);
  }

/* ==============================================================================
  _____ _____  _____         ____  _      ______ _____  
 |  __ \_   _|/ ____|  /\   |  _ \| |    |  ____|  __ \ 
 | |  | || | | (___   /  \  | |_) | |    | |__  | |  | |
 | |  | || |  \___ \ / /\ \ |  _ <| |    |  __| | |  | |
 | |__| || |_ ____) / ____ \| |_) | |____| |____| |__| |
 |_____/_____|_____/_/    \_\____/|______|______|_____/ 
                                                                                                              
============================================================================== */

  @Override
  public void disabledInit() {
    if (intake != null) {
      intake.disabledInit();
    }
    if (shooter != null) {
      shooter.disabledInit();
    }
    if (climber != null) {
      climber.disabledInit();
    }
  }

/* ==============================================================================
  _______ ______  _____ _______ 
 |__   __|  ____|/ ____|__   __|
    | |  | |__  | (___    | |   
    | |  |  __|  \___ \   | |   
    | |  | |____ ____) |  | |   
    |_|  |______|_____/   |_|   
                                                            
============================================================================== */

  @Override
  public void testInit() {
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);


  }

  @Override
  public void testPeriodic() {
    SmartDashboard.putString("MotorTesting: ", "None");
    if(drive_control.getBButton() == true) {
      frontLeftAngleEncoder.setPosition(0);
      System.out.print("Encoder 1 Reset");
      frontRightAngleEncoder.setPosition(0);
      System.out.print("Encoder 2 Reset");
      backRightAngleEncoder.setPosition(0);
      System.out.print("Encoder 3 Reset");
      backLeftAngleEncoder.setPosition(0);
      System.out.print("Encoder 4 Reset");
    }
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double rotations = SmartDashboard.getNumber("Set Rotations", 0);
    if((p != kP)) { frontLeftPidController.setP(p); kP = p; }
    if((i != kI)) { frontLeftPidController.setI(i); kI = i; }
    if((d != kD)) { frontLeftPidController.setD(d); kD = d; }
    if((iz != kIz)) { frontLeftPidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { frontLeftPidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      frontLeftPidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    if((p != kP)) { frontRightPidController.setP(p); kP = p; }
    if((i != kI)) { frontRightPidController.setI(i); kI = i; }
    if((d != kD)) { frontRightPidController.setD(d); kD = d; }
    if((iz != kIz)) { frontRightPidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { frontRightPidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      frontRightPidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    if((p != kP)) { backRightPidController.setP(p); kP = p; }
    if((i != kI)) { backRightPidController.setI(i); kI = i; }
    if((d != kD)) { backRightPidController.setD(d); kD = d; }
    if((iz != kIz)) { backRightPidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { backRightPidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      backRightPidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    if((p != kP)) { backLeftPidController.setP(p); kP = p; }
    if((i != kI)) { backLeftPidController.setI(i); kI = i; }
    if((d != kD)) { backLeftPidController.setD(d); kD = d; }
    if((iz != kIz)) { backLeftPidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { backLeftPidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      backLeftPidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

    while(drive_control.getAButton() == true) {
      SmartDashboard.putString("MotorTesting: ", "Front Left");
      frontLeftDriveMotor.set(drive_control.getLeftY());
      frontLeftAngleMotor.set(drive_control.getRightY());
    }
    while(drive_control.getBButton() == true) {
      SmartDashboard.putString("MotorTesting: ", "Front Right");
      frontRightDriveMotor.set(drive_control.getLeftY());
      frontRightAngleMotor.set(drive_control.getRightY());
    }
    while(drive_control.getXButton() == true) {   
       SmartDashboard.putString("MotorTesting: ", "Back Left");
      backLeftDriveMotor.set(drive_control.getLeftY());
      backLeftAngleMotor.set(drive_control.getRightY());
    }
    while(drive_control.getYButton() == true) {
      SmartDashboard.putString("MotorTesting: ", "Back Right");
      backRightDriveMotor.set(drive_control.getLeftY());
      backRightAngleMotor.set(drive_control.getRightY());
    }

  }
}