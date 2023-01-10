package frc.robot.testbots;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.swerve.SwerveDrive;

public class RobotSwerveOnly extends TimedRobot {

    public static final int CONTROLLER_PORT = 0;
    public static final double DEADBAND = 0.1;

    private XboxController controller;
    private SwerveDrive drive;

    @Override
    public void robotInit() {
        controller = new XboxController(CONTROLLER_PORT);
        drive = new SwerveDrive(null);
    }

    @Override
    public void disabledInit() {
        drive.stop();
    }

    @Override
    public void disabledPeriodic() {
        drive.updateWheels();
    }

    @Override
    public void autonomousInit() {
        drive.stop();
    }

    @Override
    public void autonomousPeriodic() {
        drive.updateWheels();
    }

    @Override
    public void teleopInit() {
        drive.stop();
    }

    @Override
    public void teleopPeriodic() {

        double dx = deadbandAndSquare(-controller.getLeftX());
        double dy = deadbandAndSquare(controller.getLeftY());
        double domega = deadbandAndSquare(controller.getRightX());

        drive.driveRobotRelative(dx, dy, domega);
        drive.updateWheels();
    }
    
    private double deadbandAndSquare(double val) {

        // sometimes controllers "drift" - they return a non-zero value even if
        // you're not moving the stick. this helps correct for that.
        val = MathUtil.applyDeadband(val, DEADBAND);

        // squaring changes the "curve" of drive speeds - it gives you more control
        // when you're moving the stick only slightly. note that we have to
        // preserve the sign of the value.
        return Math.abs(val) * val; 
    }

}
