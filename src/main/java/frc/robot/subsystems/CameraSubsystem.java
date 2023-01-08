package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.util.Logger;

public class CameraSubsystem {

    UsbCamera front;
    UsbCamera back;
    NetworkTableEntry cameraSelection;
    XboxController controller;
    VideoSink server;
    boolean usingFront;

    public CameraSubsystem(XboxController controller) {
        this.controller = controller;
        this.front = CameraServer.startAutomaticCapture(0);
        this.back = CameraServer.startAutomaticCapture(1);
        this.server = CameraServer.getServer();
        this.usingFront = true;
    }

    public void robotPeriodic() {
        if (usingFront) {
            server.setSource(front);
        } else {
            server.setSource(back);
        }
    }

    public void teleopPeriodic() {
        if (controller.getLeftBumperPressed()) {
            Logger.log("inverting camera; usingFront="+usingFront);
            usingFront = !usingFront;
        }
    }
}
