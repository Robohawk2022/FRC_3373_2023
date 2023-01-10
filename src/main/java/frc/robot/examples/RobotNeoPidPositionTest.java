package frc.robot.examples;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
 * https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Position%20Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java
 */
public class RobotNeoPidPositionTest extends TimedRobot {
    
    // CHANGE ME to the appropriate ID based on the testbench config
    public static final int MOTOR_CANID = 0;

    private CANSparkMax motor;
    private RelativeEncoder encoder;
    private SparkMaxPIDController pid;
    private double angle = 0.0;

    @Override
    public void robotInit() {

        motor = new CANSparkMax(MOTOR_CANID, MotorType.kBrushless);

        encoder = motor.getEncoder();
        encoder.setPositionConversionFactor(360.0);

        pid = motor.getPIDController();
        pid.setFeedbackDevice(encoder);
        pid.setOutputRange(-0.1, 0.1);
        pid.setPositionPIDWrappingEnabled(true);

        // magic defaults from vendor example code - this is what we're tuning
        pid.setP(0.1);
        pid.setI(1e-4);
        pid.setD(1.0);
        pid.setIZone(0.0);
        pid.setFF(0.0);

        // this adds properties to the dashboard for the position and PID params
        SmartDashboard.putData("Motor", builder -> {
            builder.addDoubleProperty("Angle - Actual", () -> encoder.getPosition(), null);
            builder.addDoubleProperty("Angle - Requested", () -> angle, (val) -> angle = val);
            builder.addDoubleProperty("kP", pid::getP, pid::setP);
            builder.addDoubleProperty("kI", pid::getI, pid::setI);
            builder.addDoubleProperty("kD", pid::getD, pid::setD);
            builder.addBooleanProperty("Wrapping", pid::getPositionPIDWrappingEnabled, pid::setPositionPIDWrappingEnabled);
            builder.addDoubleProperty("Max Speed", pid::getOutputMax, (val) -> pid.setOutputRange(-val, val));
        });
    }

    @Override
    public void teleopPeriodic() {
        pid.setReference(angle / 360.0, ControlType.kPosition);
    }
}
