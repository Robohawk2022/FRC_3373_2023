package frc.robot.examples;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Test robot to illustrate the features of the Neo motors that we most
 * commonly use. Use this to familiarize yourself with the API and what
 * different functions actually do.
 * 
 * First:
 * 
 *   - Read the code and see what the different options are
 * 
 * To use it:
 * 
 *   - Make sure Main.java points to this class
 *   - Make sure the motor CANID corresponds to one of the motors on the test bench
 *   - Deploy to the testbench and switch to teleop mode
 *   - Use the left joystick to run the motor forward/backward and see what happens
 *   - Observe the position and velocity on the SmartDashboard
 * 
 * Things to experiment with:
 * 
 *   - toggle motor and encoder inversion and see what they do, and how they work together
 *   - toggle neutral mode and see what that does (try to turn the motor by hand when it's stopped)
 *   - change the ramp rate and see what happens
 *   - observe the position - what happens when you make multiple rotations? does it reset to 0?
 *   - observe the velocity - how accurate is it? (use the handheld tach to measure)
 *   - change position conversion - get it report angle in degrees
 *   - change velocity conversion - get it to report "feet per second" for an imaginary wheel w/ radius 6 inches
 */
public class RobotTalonTest extends TimedRobot {

    // CHANGE ME to the appropriate ID based on the testbench config
    public static final int MOTOR_CANID = 0;

    // CHANGE ME to the appropriate port based on the laptop config
    public static final int CONTROLLER_PORT = 0;

    public static final double UNITS_PER_ROTATION = 2048.0;

    public static final double POSITION_CONVERSION = 1.0;

    public static final double VELOCITY_CONVERSION = 1.0;

    private XboxController controller;
    private TalonFX talon;

    @Override
    public void robotInit() {

        controller = new XboxController(CONTROLLER_PORT);

        // this is how you create the object which connects to the talon motor
        // control on the falcon robot
        talon = new TalonFX(MOTOR_CANID);

        // this is how you invert the motor output - reversing the notion of 
        // "forward" and "reverse" to accomodate how it's mounted on the robot
        talon.setInverted(InvertType.None);
        // talon.setInverted(InvertType.InvertMotorOutput);

        // this will set the "neutral mode" - what should the motor do when
        // it's "neutral" (i.e. not running)? brake means prevent movement, 
        // coast means allow it to spin freely.
        talon.setNeutralMode(NeutralMode.Coast);
        // talon.setNeutralMode(NeutralMode.Brake);

        // this will set the "ramp rate" - how long will the motor take to 
        // go from neutral to top speed? this is important to prevent
        // breaking things like arms that are attached to the motor.
        talon.configOpenloopRamp(0.3);

    }

    @Override
    public void robotPeriodic() {

        // this adds a property to the dashboard to track the position of
        // the motor shaft. talon measures this in "units", with 2048 units
        // per rotation. we convert this to degrees.
        SmartDashboard.putNumber("Motor Position", POSITION_CONVERSION * talon.getSelectedSensorPosition());

        // this adds a property to the dashboard to track the velocity of
        // the motor shaft (how fast it's spinning). talon measures this in
        // "units" per 100ms period. we convert this to RPM.
        SmartDashboard.putNumber("Motor Velocity", VELOCITY_CONVERSION * talon.getSelectedSensorVelocity());

    }

    @Override
    public void teleopPeriodic() {

        // gets a speed factor that ranges from -1.0 to 1.0
        double speed = controller.getLeftY();

        // applies a "deadband" - sometimes controllers will report a small joystick
        // value even if you aren't pushing them. this will filter that out.
        speed = MathUtil.applyDeadband(speed, 0.1);

        // square the input value - this makes small values smaller, which gives you
        // finer-grained control at slow speed. note that we want to preserve the
        // sign (positive or negative) of the original value.
        speed = Math.abs(speed) * speed;

        // this is one of multiple ways to tell the motor how fast to run; you are
        // giving it a percentage (from -1.0 to 1.0) of maximum output to apply
        talon.set(ControlMode.PercentOutput, speed);

    }
    
}
