package frc.robot.motors;

import com.revrobotics.CANSparkMax.ControlType;

import frc.robot.util.PIDConstant;

/**
 * Subclass of {@link AbstractClosedLoopMotor} for motors that will be spun at
 * one or more fixed velocity settings.
 * 
 * For these motors, "setting them to 0" doesn't make sense because that means
 * "actively hold the motor at 0 rpm"; it's better to just make them coast or
 * halt them.
 */
public class VelocityClosedLoopMotor extends AbstractClosedLoopMotor {
    
    // default PID values for velocity closed loop from here
    // https://github.com/REVrobotics/SPARK-MAX-Examples/blob/ef73cb1986c5af01a07bb16cb60e3754fffe3ef4/Java/Velocity%20Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java#L56
    public static final PIDConstant DEFAULT_PID = new PIDConstant(6e-5, 0, 0.0, 0.000015, 0.0, -1.0, 1.0);

    public VelocityClosedLoopMotor(String name, int port) {
        super(name, port, DEFAULT_PID);
    }

    public double getRpm() {
        return getEncoder().getVelocity();
    }

    public void setRpm(double rpm) {
        if (rpm == 0.0) {
            halt();
        } else {
            getController().setReference(rpm, ControlType.kVelocity);
        }
    }
}
