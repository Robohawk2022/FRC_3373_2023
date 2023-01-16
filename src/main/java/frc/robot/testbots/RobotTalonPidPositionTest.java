package frc.robot.testbots;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Test robot to perform calibration of position PID control.
 * 
 * To use it:
 * 
 *   - Make sure Main.java points to this class
 *   - Make sure the motor CANID corresponds to one of the motors on the test bench
 *   - Deploy to the testbench and switch to teleop mode
 *   - Use the SmartDashboard to observe and set the target motor angle
 *   - Adjust gain parameters appropriately
 * 
 * https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/tree/master/Java%20Talon%20FX%20(Falcon%20500)/PositionClosedLoop/src/main/java/frc/robot
 */
public class RobotTalonPidPositionTest extends TimedRobot {
    
    // CHANGE ME to the appropriate ID based on the testbench config
    public static final int MOTOR_CANID = 0;
    public static final int ENCODER_CANID = 0;

    // I think this is correct, based on the special encoder we're using
    public static final double UNITS_PER_ROTATION = 4096.0;

    // We need a number we can multiple by "ticks" to get "degrees"
    public static final double POS_CONVERSION = 1.0;

    private CANCoder encoder;
    private TalonFX talon;
    private double kP;
    private double kI;
    private double kD;
    private double angle;
    private double maxOutput;

    @Override
    public void robotInit() {

        encoder = new CANCoder(ENCODER_CANID);

        talon = new TalonFX(MOTOR_CANID);
        talon.configRemoteFeedbackFilter(encoder, 0);
        talon.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
        talon.configSoftLimitDisableNeutralOnLOS(false, 0);
        talon.setInverted(InvertType.None);
        talon.setNeutralMode(NeutralMode.Coast);
        talon.configClosedloopRamp(0.3);

        // magic defaults from vendor example code - this is what we're tuning
        setP(0.15);
        setI(0.0);
        setD(1.0);
        setMaxOutput(1.0);

        // this adds properties to the dashboard for the position and PID params
        SmartDashboard.putData("Motor", builder -> {
            builder.addDoubleProperty("Angle - Actual", () -> talon.getSelectedSensorPosition() * POS_CONVERSION, null);
            builder.addDoubleProperty("Angle - Requested", () -> angle, val -> angle = val);
            builder.addDoubleProperty("kP", () -> kP, this::setP);
            builder.addDoubleProperty("kI", () -> kI, this::setI);
            builder.addDoubleProperty("kD", () -> kD, this::setD);
            builder.addDoubleProperty("Max Output", () -> maxOutput, this::setMaxOutput);
            builder.addDoubleProperty("Position", () -> talon.getSelectedSensorPosition(), null);
        });
    }

    private void setP(double val) {
        kP = val;
        talon.config_kP(0, val);
    }

    private void setI(double val) {
        kI = val;
        talon.config_kI(0, val);
    }

    private void setD(double val) {
        kD = val;
        talon.config_kD(0, val);
    }

    private void setMaxOutput(double val) {
        maxOutput = val;
        talon.configPeakOutputForward(val);
        talon.configPeakOutputReverse(-val);
    }

    @Override
    public void teleopPeriodic() {
        double units = angle / POS_CONVERSION;
        talon.set(ControlMode.Position, units);
    }
}
