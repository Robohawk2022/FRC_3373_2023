package frc.robot.testbots;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.swerve.old.MacDrive;

public class RobotMacDriveOnly extends TimedRobot {

    public static final int CONTROLLER_PORT = 0;

    private MacDrive drive;

    @Override
    public void robotInit() {
        drive = new MacDrive(new XboxController(CONTROLLER_PORT));
    }

    @Override
    public void robotPeriodic() {
        drive.updateStats();
    }

    @Override
    public void disabledInit() {
        drive.stop();
    }

    @Override
    public void autonomousInit() {
        drive.stop();
    }

    @Override
    public void teleopInit() {
        drive.stop();
    }

    @Override
    public void teleopPeriodic() {
        drive.updateWheels();
    }
    
    @Override
    public void testInit() {
        drive.startTest();
    }

    @Override
    public void testPeriodic() {
        drive.testPeriodic();
    }
}
