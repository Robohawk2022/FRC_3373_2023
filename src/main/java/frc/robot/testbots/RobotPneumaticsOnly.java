package frc.robot.testbots;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class RobotPneumaticsOnly extends TimedRobot {
    
    private XboxController controller;
    private Solenoid pressureReleaseLo;
    private Solenoid pressureReleaseHi;
    private Solenoid clampExtend;
    private Solenoid clampRelease;

    @Override
    public void robotInit() {
        controller = new XboxController(0);
        pressureReleaseLo = new Solenoid(PneumaticsModuleType.REVPH, 14);
        pressureReleaseHi = new Solenoid(PneumaticsModuleType.REVPH, 14);
        clampExtend = new Solenoid(PneumaticsModuleType.REVPH, 6);
        clampRelease = new Solenoid(PneumaticsModuleType.REVPH, 7);
    }

    @Override
    public void teleopPeriodic() {
        
        // use the buttons to toggle the invidual solenoids

        if (controller.getAButton()) {
            pressureReleaseLo.toggle();
        }
        
        if (controller.getBButton()) {
            pressureReleaseHi.toggle();
        }
        
        if (controller.getXButton()) {
            clampExtend.toggle();
        }
        
        if (controller.getYButton()) {
            clampRelease.toggle();
        }

        // try out various combinations of these things

        if (controller.getLeftBumperPressed()) {
            extendLo();
        }

        if (controller.getRightBumperPressed()) {
            extendHi();
        }

        if (controller.getStartButtonPressed()) {
            setNeutral();
        }
    }

    public void setNeutral() {
        pressureReleaseLo.set(false);
        pressureReleaseHi.set(false);
        clampExtend.set(false);
        clampRelease.set(false);
    }

    public void extendLo() {
        pressureReleaseLo.set(true);
        pressureReleaseHi.set(false);
        clampExtend.set(true);
        clampRelease.set(false);
    }

    public void extendHi() {
        pressureReleaseLo.set(true);
        pressureReleaseHi.set(false);
        clampExtend.set(true);
        clampRelease.set(false);
    }
}
