package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.motors.MotorFactory;
import frc.robot.motors.PositionClosedLoopMotor;
import frc.robot.motors.VelocityClosedLoopMotor;
import frc.robot.util.Logger;

/**
 * Subsystem for shooting
 */
public class ShooterSubsystem {

    /** Minimum speed for the launch wheel to avoid a ball getting stuck */
    public static final double MINIMUM_LAUNCH_RPM = -2000;

    /** Because sometimes DigitalInput switches are reversed */
    public static final boolean BALL_AVAILABLE_PRESSED = true;

    /** Starting speed of the launch wheel */
    public static final double STARTING_LAUNCH_RPM = -12000;

    /** Maximum speed at which to rotate the indexer */
    public static final double INDEXER_MAX_SPEED = 0.5;

    /** How many rotations does the indexer need to lock in a ball? */
    public static final double LOCKIN_ROTATIONS = 14;

    /** How many rotations does the indexer need to push out a ball? */
    public static final double SHOOT_ROTATIONS = 40;

    private final XboxController controller;
    private final DigitalInput ballSensor;
    private final VelocityClosedLoopMotor launchWheel;
    private final PositionClosedLoopMotor indexerWheel;
    private boolean spinLaunchWheel;
    private int indexerResetCount;
    private double targetLaunchSpeed;
    private int autoShotsPending;
    private long teleopRounds;

    public ShooterSubsystem(XboxController controller, 
            int launchMotorPort, 
            int indexerMotorPort, 
            int ballAvailableSwitchPort) {
        System.err.println("initializing shooter");
        this.controller = controller;
        this.ballSensor = new DigitalInput(ballAvailableSwitchPort);
        this.launchWheel = MotorFactory.makeVelocityClosedLoopMotor("Launch", launchMotorPort);
        this.indexerWheel = MotorFactory.makePositionClosedLoopMotor("Indexer", indexerMotorPort);
        indexerWheel.setMaxSpeed(INDEXER_MAX_SPEED);
        autoShotsPending = 2;
        disabledInit();
    }

    public void setLaunchWheelEnabled(boolean enabled) {
        Logger.log("shooter: setting launch wheel to enabled=", enabled);
        spinLaunchWheel = enabled;
    }

    public void shoot() {
        boolean atSpeed = launchWheel.getRpm() < MINIMUM_LAUNCH_RPM; // yes, less than, because speeds are negative
        if (atSpeed) {
            Logger.log("shooter: shooting!");
            indexerWheel.rotate(SHOOT_ROTATIONS);    
        } else {
            Logger.log("shooter: refusing to shoot; not at speed");
        }
    }

    // called 50x per second, no matter what mode we're in
    public void robotPeriodic() {
        SmartDashboard.putBoolean("Launch Spinning?", spinLaunchWheel);
        SmartDashboard.putNumber("Launch Target RPM", targetLaunchSpeed);
        SmartDashboard.putNumber("Launch Current RPM", launchWheel.getRpm());
        SmartDashboard.putNumber("Indexer Target Pos", indexerWheel.getTargetPosition());
        SmartDashboard.putNumber("Indexer Current Pos", indexerWheel.getPosition());
        SmartDashboard.putNumber("Shooter Counter", teleopRounds);
        SmartDashboard.putBoolean("Ball Sensor", ballSensor.get());
    }

    // called when the robot is put into disabled mode
    public void disabledInit() {
        spinLaunchWheel = false;
        targetLaunchSpeed = STARTING_LAUNCH_RPM;
        indexerResetCount = 0;
        launchWheel.halt();
        indexerWheel.resetClosedLoopControl();
    }

    // ================================================================
    // SINGLE SHOOTER
    // Autonomous routine that shoots a single ball after a while
    // ================================================================

    public void singleShooterInit() {
        autoShotsPending = 1;
        Logger.log("shooter: starting single shooter");
    }

    public void singleShooterPeriodic(double seconds) {
        if (seconds > 2.0) {
            if (!spinLaunchWheel) {
                setLaunchWheelEnabled(true);
            }
            if (seconds > 10.5 && autoShotsPending > 1) {
                shoot();
                autoShotsPending--;
            }
            if (seconds > 12) {
                setLaunchWheelEnabled(false);
            }    
        }
        updateLaunchWheel();
        updateIndexerWheel();
    }

    // ================================================================
    // DOUBLE SHOOTER
    // Start routine that shoots two balls after a while
    // ================================================================

    public void doubleShooterInit() {
        autoShotsPending = 2;
        Logger.log("shooter: starting double shooter");
    }

    public void doubleShooterPeriodic(double seconds) {
        if (seconds > 2.0) {
            if (!spinLaunchWheel) {
                setLaunchWheelEnabled(true);
            }
            if (seconds > 10.5 && autoShotsPending > 1) {
                shoot();
                autoShotsPending--;
            }
            if (seconds > 12.5 && autoShotsPending > 0) {
                shoot();
                autoShotsPending--;
            }
            if (seconds > 14) {
                setLaunchWheelEnabled(false);
            }         
        }
        updateLaunchWheel();
        updateIndexerWheel();
    }
    
    public void teleopInit() {
        teleopRounds = 0L;
    }

    // called 50x per second in teleop mode
    public void teleopPeriodic() {
        teleopRounds++;
        updateLaunchWheel();
        updateIndexerWheel();
    }

    /**
     * Updates the launch wheel - turning it on/off and raising/lowering
     * the target speed.
     */
    protected void updateLaunchWheel() {

        // start button toggles the launch wheel on and off
        if (controller.getStartButtonPressed()) {
            setLaunchWheelEnabled(!spinLaunchWheel);
        }

        // if the launch wheel is spinning, we'll allow speed changes
        if (spinLaunchWheel) {

            // A button: reset launch speed
            if (controller.getAButtonPressed()) {
                targetLaunchSpeed = STARTING_LAUNCH_RPM;
                Logger.log("shooter: reset launch wheel to ", targetLaunchSpeed);
            }
            // X button: go 10% slower
            else if (controller.getXButtonPressed()) {
                targetLaunchSpeed *= 0.975;
                Logger.log("shooter: slowed down launch wheel to ", targetLaunchSpeed);
            }
            // Y button: go 10% faster
            else if (controller.getYButtonPressed()) {
                targetLaunchSpeed *= 1.025;
                Logger.log("shooter: sped up launch wheel to ", targetLaunchSpeed);
            }

            launchWheel.setRpm(targetLaunchSpeed);
        }
        else {
            launchWheel.coast();
        }
    }

    /**
     * Updates the indexer wheel - turning it to fire or intake a ball.
     */
    public void updateIndexerWheel() {

        // if the left bumper is held, someone wants to rotate the index wheel
        // backwards a skoche. disabled closed loop control and do it. this is
        // the only thing we'll do.
        if (controller.getLeftBumper()) {
            indexerWheel.set(-INDEXER_MAX_SPEED / 5.0);
            indexerResetCount = 50;
            return;
        }

        // if our last loop was running the indexer backwards, re-enable
        // closed loop control
        if (indexerResetCount > 0) {
            indexerWheel.halt();
            if (indexerResetCount == 1) {
                indexerWheel.resetClosedLoopControl();
            }
            indexerResetCount -= 1;
            return;
        }

        // trigger indexing IF there is a ball in front of the sensor now,
        // and there wasn't one last time
        if (ballSensor.get()) {
            Logger.log("shooter: rotating for intake");
            indexerWheel.rotate(LOCKIN_ROTATIONS);
        }

        // if someone wants to shoot, and the wheel's at speed and we 
        // have a ball, go for it!
        if (controller.getBButtonPressed()) {
            shoot();
        }

        indexerWheel.updateSpeed();
    }
}