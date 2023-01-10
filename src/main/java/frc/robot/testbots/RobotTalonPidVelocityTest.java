package frc.robot.testbots;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

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
 *   - Use the SmartDashboard to observe and set the target motor speed
 *   - Adjust gain parameters appropriately
 * 
 * https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20Talon%20FX%20(Falcon%20500)/VelocityClosedLoop_ArbFeedForward/src/main/java/frc/robot
 */
public class RobotTalonPidVelocityTest extends TimedRobot {
    
    // CHANGE ME to the appropriate ID based on the testbench config
    public static final int MOTOR_CANID = 0;

    public static final double UNITS_PER_ROTATION = 2048.0;

    public static final double VEL_CONVERSION = 600.0 / 2048.0;

    private TalonFX talon;
    private double kP;
    private double kI;
    private double kD;
    private double kIZ;
    private double kF;
    private double maxOutput;
    private double rpm = 0.0;

    @Override
    public void robotInit() {

        talon = new TalonFX(MOTOR_CANID);
        // TODO - wrapping?

        // magic defaults from vendor example code - this is what we're tuning
        setP(0.1);
        setI(0.001);
        setIZ(300.0);
        setD(5.0);
        setF(1023.0 / 20660.0);
        setMaxOutput(1.0);

        // this adds properties to the dashboard for the position and PID params
        SmartDashboard.putData("Motor", builder -> {
            builder.addDoubleProperty("kP", () -> kP, this::setP);
            builder.addDoubleProperty("kI", () -> kI, this::setI);
            builder.addDoubleProperty("kIZ", () -> kIZ, this::setIZ);
            builder.addDoubleProperty("kD", () -> kD, this::setD);
            builder.addDoubleProperty("kF", () -> kF, this::setF);
            builder.addDoubleProperty("Max Output", () -> maxOutput, this::setMaxOutput); 
            builder.addDoubleProperty("RPM - Actual", () -> talon.getSelectedSensorVelocity() * VEL_CONVERSION, null);
            builder.addDoubleProperty("RPM - Requested", () -> rpm, val -> rpm = val);
            builder.addDoubleProperty("Velocity", talon::getSelectedSensorVelocity, null);
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

    private void setIZ(double val) {
        kIZ = val;
        talon.config_IntegralZone(0, kIZ);
    }

    private void setD(double val) {
        kD = val;
        talon.config_kD(0, val);
    }

    private void setF(double val) {
        kF = val;
        talon.config_kF(0, val);
    }

    private void setMaxOutput(double val) {
        maxOutput = val;
        talon.configPeakOutputForward(val);
        talon.configPeakOutputReverse(-val);
    }

    @Override
    public void teleopPeriodic() {
        double units = rpm / VEL_CONVERSION;
        talon.set(ControlMode.Velocity, units);
    }
}
