package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.motors.MotorFactory;
import frc.robot.motors.VelocityClosedLoopMotor;
import frc.robot.util.Logger;

/**
 * Subsystem for ball intake 
 */
public class IntakeSubsystem {

    /** Starting value for the speed of the intake wheel */
    public static final double STARTING_RPM = 14000;

    /** Speed value for dropping the frame (rotates in reverse) */
    public static final double DROP_FRAME_SPEED = -0.03;

    /** Amount of time for dropping the frame */
    public static final double DROP_FRAME_SECONDS = 5.0;
    
    private final XboxController controller;
    private final VelocityClosedLoopMotor intakeMotor;
    private boolean spinWheel;
    private double targetSpeed;
    private long teleopRounds;

    public IntakeSubsystem(XboxController controller, int intakeMotorPort) {
        this.controller = controller;
        this.intakeMotor = MotorFactory.makeVelocityClosedLoopMotor("Intake", intakeMotorPort);
        disabledInit();
    }

    // called 50x per second, no matter what mode we're in
    public void robotPeriodic() {
        SmartDashboard.putNumber("Intake Target RPM", targetSpeed);
        SmartDashboard.putNumber("Intake Current RPM", intakeMotor.getRpm());
        SmartDashboard.putBoolean("Intake Spinning?", spinWheel);
        SmartDashboard.putNumber("Intake Counter", teleopRounds);
    }
    
    // called when the robot is put into disabled mode
    public void disabledInit() {
        Logger.log("intake: putting intake system in disabled mode");
        spinWheel = false;
        targetSpeed = STARTING_RPM;
        intakeMotor.halt();
    }

    // ================================================================
    // SINGLE SHOOTER
    // Autonomous routine that drops the frame only
    // ================================================================

    public void singleShooterInit() {
        Logger.log("intake: starting single shooter");
    }

    public void singleShooterPeriodic(double seconds) {
        if (seconds < 3.0) {
            intakeMotor.set(DROP_FRAME_SPEED);
        } else if (seconds < DROP_FRAME_SECONDS) {
            intakeMotor.set(5 * DROP_FRAME_SPEED);
        } else {
            intakeMotor.coast();
        }
    }

    // ================================================================
    // DOUBLE SHOOTER
    // Autonomous routine that drops the frame and picks up a ball
    // ================================================================

    public void doubleShooterInit() {
        Logger.log("intake: starting double shooter");
    }

    public void doubleShooterPeriodic(double seconds) {
        if (seconds < 3.0) {
            intakeMotor.set(DROP_FRAME_SPEED);
        } else if (seconds < DROP_FRAME_SECONDS) {
            intakeMotor.set(5 * DROP_FRAME_SPEED);
        } else if (seconds < 10.0) {
            intakeMotor.setRpm(STARTING_RPM);
        } else {
            intakeMotor.coast();
        }
    }

    public void teleopInit() {
        teleopRounds = 0L;
    }

    // called 50x per second in teleop mode
    public void telopPeriodic() {

        teleopRounds++;
        
        // back button turns the wheel on and off
        if (controller.getBackButtonPressed()) {
            spinWheel = !spinWheel;
            Logger.log("intake: toggled intake wheel to ", spinWheel);
        }

        // if the wheel is spinning, we'll allow speed changes
        if (spinWheel) {

            // hold both bumpers: reset target speed
            if (controller.getAButtonPressed()) {
                targetSpeed = STARTING_RPM;
                Logger.log("intake: reset intake wheel to ", targetSpeed);
            }
            // press left bumper: go 10% slower
            else if (controller.getXButtonPressed()) {
                targetSpeed *= 0.9;
                Logger.log("intake: slowed down intake wheel to ", targetSpeed);
            }
            // press right bumper: go 10% faster
            else if (controller.getYButton()) {
                targetSpeed *= 1.1;
                Logger.log("intake: sped up intake wheel to ", targetSpeed);
            }

            // hold left trigger: reverse the wheel
            if (controller.getLeftTriggerAxis() > 0.5) {
                intakeMotor.setRpm(-targetSpeed);
            }
            else {
                intakeMotor.setRpm(targetSpeed);
            }       
        }
        else {
            intakeMotor.coast();
        }
    }
}
