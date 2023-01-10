package frc.robot.swerve;

import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.swerve.SwerveConfig.*;

/**
 * Represents the collection of four swerve wheels and drives them using
 * swerve kinematics.
 * 
 * Kinematics are the calculations that turn a desired outcome for the
 * overall chassis of the robot into instructions for each individual
 * wheel.
 * 
 * https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervebot
 */
public class SwerveDrive {

    private final Supplier<Rotation2d> gyro;
    private final SwerveDriveKinematics kinematics;
    private final List<SwerveWheel> wheels;
    private double maxDriveSpeed;

    public SwerveDrive(Supplier<Rotation2d> gyro) {

        this.gyro = gyro;
        this.maxDriveSpeed = DEFAULT_MAX_DRIVE_SPEED_FPS;
        this.kinematics = new SwerveDriveKinematics(calculateWheelPositions());
        this.wheels = Arrays.asList(
            new SwerveWheel("FL", FL_DRIVE_CANID, FL_TURN_CANID, FL_REVERSE),            
            new SwerveWheel("FR", FR_DRIVE_CANID, FR_TURN_CANID, FR_REVERSE),            
            new SwerveWheel("BL", BL_DRIVE_CANID, BL_TURN_CANID, BL_REVERSE),        
            new SwerveWheel("BR", BR_DRIVE_CANID, BR_TURN_CANID, BR_REVERSE));

        SmartDashboard.putData("Swerve Drive", builder -> {
            builder.addDoubleProperty("Max Speed", () -> maxDriveSpeed, val -> maxDriveSpeed = val);
        });
    }

    public void driveRobotRelative(double dx, double dy, double domega) {
        drive(new ChassisSpeeds(dx, dy, domega));
    }

    public void driveFieldRelative(double dx, double dy, double domega) {
        drive(ChassisSpeeds.fromFieldRelativeSpeeds(dx, dy, domega, gyro.get()));

    }

    public void stop() {
        for (int i=0; i<wheels.size(); i++) {
            wheels.get(i).stop();
        }
    }

    public void drive(ChassisSpeeds speeds) {

        // turn the requested chassis movement into desired states for each wheel
        SwerveModuleState [] states = kinematics.toSwerveModuleStates(speeds);

        // sometimes that calculation can result in wheels running faster than
        // desired. this will normalize them, but preserve their ratio so we
        // don't get out of position.
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxDriveSpeed);

        for (int i=0; i<wheels.size(); i++) {
            wheels.get(i).setDesiredState(states[i]);
        }
    }

    public void updateWheels() {
        for (int i=0; i<wheels.size(); i++) {
            wheels.get(i).update();
        }
    }
}
