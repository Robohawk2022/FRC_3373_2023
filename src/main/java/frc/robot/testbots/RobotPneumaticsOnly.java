package frc.robot.testbots;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class RobotPneumaticsOnly extends TimedRobot {
    
    public static final int REVPH_CANID = 5;

    private XboxController controller;
    private Solenoid pressureReleaseLo;
    private Solenoid pressureReleaseHi;
    private Solenoid clampExtend;
    private Solenoid clampRelease;
    private boolean prl;
    private boolean prh;
    private boolean ext;
    private boolean rel;

    @Override
    public void robotInit() {

        controller = new XboxController(0);

        // TODO - double check that the last parameter (channel assignment) is correct
        // notice: it's a channel ID on the pneumatics hub, not a CAN ID
        pressureReleaseLo = new Solenoid(REVPH_CANID, PneumaticsModuleType.REVPH, 14);
        pressureReleaseHi = new Solenoid(REVPH_CANID, PneumaticsModuleType.REVPH, 14);
        clampExtend = new Solenoid(REVPH_CANID, PneumaticsModuleType.REVPH, 6);
        clampRelease = new Solenoid(REVPH_CANID, PneumaticsModuleType.REVPH, 7);

        prl = false;
        prh = false;
        ext = false;
        rel = false;
    }

    @Override
    public void teleopPeriodic() {
        
        // A will toggle the low-pressure release solenoid
        if (controller.getAButton()) {
            prl = !prl;
        }
        
        // B will toggle the high-pressure release solenoid
        if (controller.getBButton()) {
            prh = !prh;
        }
        
        // X will toggle the extension solenoid
        if (controller.getXButton()) {
            ext = !ext;
        }
        
        // Y will toggle the release solenoid
        if (controller.getYButton()) {
            clampRelease.toggle();
        }

        // TODO try out various combinations of these things, and see which ones work
        if (controller.getLeftBumperPressed()) {
            extendLo();
        }

        if (controller.getRightBumperPressed()) {
            extendHi();
        }

        if (controller.getStartButtonPressed()) {
            setNeutral();
        }

        // TODO do solenoids need to be told what to do during every loop, like motors do?
        pressureReleaseLo.set(prl);
        pressureReleaseHi.set(prh);
        clampExtend.set(ext);
        clampRelease.set(rel);

    }

    public void setNeutral() {
        prl = false;
        prh = false;
        ext = false;
        rel = false;
    }

    public void extendLo() {
        prl = true;
        prh = false;
        ext = true;
        rel = false;
    }

    public void extendHi() {
        prl = true;
        prh = false;
        ext = true;
        rel = false;
    }
}
