package frc.robot.testbots;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class RobotNavxOnly extends TimedRobot {

    public static final int CONTROLLER_PORT = 0;

    private AHRS navx;
    private boolean lastCalibrating = false;
    private double lastHeading = 9999999;
    private double lastPitch = 9999999;
    private double lastX = 9999999;
    private double lastY = 9999999;
    private XboxController controller;

    @Override
    public void robotInit() {

        navx = new AHRS();
        controller = new XboxController(CONTROLLER_PORT);

        updateHeading();
        updatePitch();
        updateDisplacement();
    }

    @Override
    public void robotPeriodic() {
        updateCalibrating();
    }

    @Override
    public void teleopPeriodic() {

        if (controller.getPOV() == 0) {
            System.err.println("requesting calibration");
            navx.calibrate();
        }

        if (controller.getBButtonPressed()) {
            System.err.println("resetting displacement");
            navx.resetDisplacement();
        }

        updateHeading();
        updatePitch();
        updateDisplacement();
    }

    private void updateCalibrating() {
        boolean nextCalibrating = navx.isCalibrating();
        if (lastCalibrating != nextCalibrating) {
            System.err.println(nextCalibrating ? "calibration started" : "calibration done");
            lastCalibrating = nextCalibrating;
        }
    }

    private void updateHeading() {
        double nextHeading = navx.getCompassHeading();
        if (Math.abs(nextHeading - lastHeading) > 1.0) {
            System.err.println(String.format("heading = %.2f", nextHeading));
            lastHeading = nextHeading;
        }
    }

    private void updatePitch() {
        double nextPitch = navx.getPitch();
        if (Math.abs(nextPitch - lastPitch) > 1.0) {
            System.err.println(String.format("pitch = %.2f", nextPitch));
            lastPitch = nextPitch;
        }
    }

    private void updateDisplacement() {
        double nextX = Units.metersToFeet(navx.getDisplacementX());
        double nextY = Units.metersToFeet(navx.getDisplacementY());
        if (distance(lastX, lastY, nextX, nextY) > 1.0) {
            System.err.println(String.format("position = (%.2f, %.2f)", nextX, nextY));
            lastX = nextX;
            lastY = nextY;
        }
    }

    private double distance(double x1, double y1, double x2, double y2) {
        double dx = x1 - x2;
        double dy = y1 - y2;
        return Math.sqrt(dx * dx + dy * dy);
    }
}
