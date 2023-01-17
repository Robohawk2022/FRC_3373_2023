package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

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

    /**
     * Creates a drive motor for the swerve drive. Drive motors are used with
     * velocity-based PID control.
     */
    public static TalonFX createDriveMotor(int canId, boolean reverseMotor){
        TalonFX driveMotor = new TalonFX(canId);
        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        driveMotor.configSoftLimitDisableNeutralOnLOS(true, canId);
        if (reverseMotor) {
            driveMotor.setInverted(InvertType.None);
        } else {
            driveMotor.setInverted(InvertType.InvertMotorOutput);
        }
        driveMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.configClosedloopRamp(0.3);
        driveMotor.config_kP(0, 0.1);
        driveMotor.config_kI(0, 0.001);
        driveMotor.config_kD(0, 5.0);
        driveMotor.config_kF(0, 1023.0 / 20660.0);
        driveMotor.config_IntegralZone(0, 300.0);
        driveMotor.configPeakOutputForward(1.0);
        driveMotor.configPeakOutputReverse(-1.0);
        driveMotor.configClosedloopRamp(0.3);
        driveMotor.configOpenloopRamp(0.3);
        return driveMotor;
    }

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
