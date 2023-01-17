package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.PIDConstant;

public class SwerveConfig {

    // physical attributes of the chassis
    // TODO update with correct measurements for our build

    public static final double WHEEL_BASE_INCHES = 40;
    public static final double TRACK_WIDTH_INCHES = 20;
    public static final double WHEEL_DIAM_INCHES = 6;

    // behavior configuration for driving

    public static final double DEFAULT_MAX_DRIVE_SPEED_FPS = 2.5;

    // low-level configuration for drive motors
    // TODO replace PID parameters with ones tuned for our build

    public static final NeutralMode DEFAULT_DRIVE_NEUTRAL_MODE = NeutralMode.Brake;
    public static final double DEFAULT_DRIVE_RAMP_RATE = 0.3;
    public static final PIDConstant DRIVE_PID = new PIDConstant(
        0.1, 0.01, 5.0, 1023.0 / 20660.0, 300.0, -1.0, 1.0);

    // low-level configuration for turn motors
    // TODO replace PID parameters with ones tuned for our build

    public static final NeutralMode DEFAULT_TURN_NEUTRAL_MODE = NeutralMode.Brake;
    public static final double DEFAULT_TURN_RAMP_RATE = 0.3;
    public static final PIDConstant TURN_PID = new PIDConstant(
        0.15, 0.0, 1.0, 1.0);

    // CANID assignments and mounting positions for motors
    // TODO update with correct assignments for our build

    public static final int FL_DRIVE_CANID = 10;
    public static final int FL_TURN_CANID = 11;
    public static final boolean FL_REVERSE = false;

    public static final int FR_DRIVE_CANID = 13;
    public static final int FR_TURN_CANID = 14;
    public static final boolean FR_REVERSE = false;

    public static final int BL_DRIVE_CANID = 16;
    public static final int BL_TURN_CANID = 17;
    public static final boolean BL_REVERSE = false;
    
    public static final int BR_DRIVE_CANID = 19;
    public static final int BR_TURN_CANID = 20;
    public static final boolean BR_REVERSE = false;

    // helper function






    ///////////////////////////////////////////// UNDERCONSTRUCTION




    //The createDriveMotor method 

    public static void createDriveMotor(int canld, boolean reverseMotor){

        TalonFX DriveMotor = new TalonFX(canld);
        {
          DriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
          DriveMotor.configSoftLimitDisableNeutralOnLOS(true, canld);
          DriveMotor.setInverted(InvertType.None);
          DriveMotor.setNeutralMode(NeutralMode.Brake);
          DriveMotor.configClosedloopRamp(0.3);
          DriveMotor.config_kP(0, 0.1);
          DriveMotor.config_kI(0, 0.001);
          DriveMotor.config_kD(0, 5.0);
          DriveMotor.config_kF(0, 1023.0 / 20660.0);
          DriveMotor.config_IntegralZone(0, 300.0);
          DriveMotor.configMaxIntegralAccumulator(0, 1.0);
        }
  
    }
    

    /////////////////////////////////////////////// UNDERCONSTRUCTION








    public static Translation2d [] calculateWheelPositions() {

        double lm = Units.inchesToMeters(WHEEL_BASE_INCHES) / 2.0;
        double wm = Units.inchesToMeters(TRACK_WIDTH_INCHES) / 2.0;

        Translation2d [] positions = new Translation2d[4];
        positions[0] = new Translation2d(wm, lm);     // front left
        positions[1] = new Translation2d(-wm, lm);    // front right
        positions[2] = new Translation2d(wm, -lm);    // back left
        positions[3] = new Translation2d(-wm, -lm);   // back right
        return positions;
    }
}
