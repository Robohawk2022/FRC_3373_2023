package frc.robot.motors;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.system.plant.DCMotor;

/**
 * This is a handy way to create new motors, because it remembers all the motors you
 * make and provides an easy way to update the dashboard and initialize simulated
 * motors, if that's your kind of thing.
 */
public class MotorFactory {

    private static final List<NamedMotor> ALL_MOTORS = new ArrayList<>();

    public static VelocityClosedLoopMotor makeVelocityClosedLoopMotor(String name, int port) {
        VelocityClosedLoopMotor motor = new VelocityClosedLoopMotor(name, port);
        ALL_MOTORS.add(motor);
        return motor;
    }

    public static PositionClosedLoopMotor makePositionClosedLoopMotor(String name, int port) {
        PositionClosedLoopMotor motor = new PositionClosedLoopMotor(name, port);
        ALL_MOTORS.add(motor);
        return motor;
    }

    public static NamedMotor makeNamedMotor(String name, int port) {
        NamedMotor motor = new NamedMotor(name, port);
        ALL_MOTORS.add(motor);
        return motor;
    }

    public static void updateDashboard() {
        for (NamedMotor motor : ALL_MOTORS) {
           motor.updateDashboard();
        }
    }

    public static void simulationInit() {
        for (NamedMotor motor : ALL_MOTORS) {
            CANSparkMax canSpark = motor.getMotor();
            DCMotor simMotor = DCMotor.getNEO(canSpark.getDeviceId());
            REVPhysicsSim.getInstance().addSparkMax(canSpark, simMotor);
        }
    }
}
