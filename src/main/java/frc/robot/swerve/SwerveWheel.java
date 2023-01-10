package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Represents a single swerve wheel and its two related motors. Knows how to
 * turn the wheel to a requested angle and run it at a requested rate. 
 */
public class SwerveWheel {

    // 90 degree sideways angle
    public static final Rotation2d SIDEWAYS = Rotation2d.fromDegrees(90);

    // This is how many encoder units our motors have in a single revolution.
    public static final double UNITS_PER_REVOLUTION = 2048.0;

    // This is how many meters the wheel travels in a single rotation.
    public static final double METERS_PER_REVOLUTION = 
            Math.PI * Units.inchesToMeters(SwerveConfig.WHEEL_DIAM_INCHES);

    // We like dealing with rotation in degrees. Our motors calculate their
    // position with units. This is the conversion factor - multiply the
    // encoder signal by this to get degrees, and vice versa.
    public static final double POS_CONVERSION = 360.0 / UNITS_PER_REVOLUTION;

    // WPI Lib calculates speed in meters per second. Our motors calculate it in
    // units per 100ms interval. This is the conversion factor - multiply the
    // encoder signal by this to get mps, and vice versa.
    public static final double VEL_CONVERSION = 
            10.0 * METERS_PER_REVOLUTION / UNITS_PER_REVOLUTION;

    private final SwerveModuleState currentState;
    private final SwerveModuleState desiredState;
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;

    public SwerveWheel(String name, int drivePort, int turnPort, boolean inverted) {

        this.currentState = new SwerveModuleState();
        this.desiredState = currentState;

        this.driveMotor = new TalonFX(drivePort);
        this.driveMotor.setNeutralMode(SwerveConfig.DEFAULT_DRIVE_NEUTRAL_MODE);
        this.driveMotor.setInverted(inverted ? InvertType.InvertMotorOutput : InvertType.None);
        this.driveMotor.configClosedloopRamp(SwerveConfig.DEFAULT_DRIVE_RAMP_RATE);
        SwerveConfig.DRIVE_PID.configPID(driveMotor);

        this.turnMotor = new TalonFX(turnPort);
        this.turnMotor.setNeutralMode(SwerveConfig.DEFAULT_TURN_NEUTRAL_MODE);
        this.turnMotor.setInverted(InvertType.None);
        this.turnMotor.configClosedloopRamp(SwerveConfig.DEFAULT_TURN_RAMP_RATE);
        SwerveConfig.TURN_PID.configPID(turnMotor);

        SmartDashboard.putData("Swerve Wheel ("+name+")", builder -> {
            builder.addDoubleProperty("Current Angle (deg)", currentState.angle::getDegrees, null);
            builder.addDoubleProperty("Desired Angle (deg)", desiredState.angle::getDegrees, null);
            builder.addDoubleProperty("Current Speed (fps)", () -> Units.metersToFeet(currentState.speedMetersPerSecond), null);
            builder.addDoubleProperty("Desired Speed(fps)", () -> Units.metersToFeet(desiredState.speedMetersPerSecond), null);
        });
    }

    /**
     * Stops the motors from spinning
     */
    public void stop() {
        desiredState.speedMetersPerSecond = 0.0;
        driveMotor.set(ControlMode.PercentOutput, 0.0);
        turnMotor.set(ControlMode.PercentOutput, 0.0);
    }

    /**
     * Puts the wheels in a 90 degrees sideways rotation
     */
    public void sideways() {
        desiredState.angle = SIDEWAYS;
    }

    /**
     * Sets the desired state for this wheel. This does NOT move the wheel; that
     * happens in the update loop.
     */
    public void setDesiredState(SwerveModuleState state) {

        // optimize the state to avoid turning through 90 degrees
        state = SwerveModuleState.optimize(state, currentState.angle);

        this.desiredState.speedMetersPerSecond = state.speedMetersPerSecond;
        this.desiredState.angle = state.angle;
    }
    
    /**
     * This must be called in a control loop in order to move the wheel to the
     * correct position.
     */
    public void update() {

        driveMotor.set(ControlMode.Velocity, desiredState.speedMetersPerSecond / VEL_CONVERSION);
        turnMotor.set(ControlMode.Position, desiredState.angle.getDegrees() / POS_CONVERSION);

        currentState.speedMetersPerSecond = VEL_CONVERSION * driveMotor.getSelectedSensorVelocity();
        currentState.angle = Rotation2d.fromDegrees(POS_CONVERSION * turnMotor.getSelectedSensorPosition());
    }
}
