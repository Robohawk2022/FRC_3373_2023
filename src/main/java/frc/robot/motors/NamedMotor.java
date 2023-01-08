package frc.robot.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Just a named wrapper around a {@link CANSparkMax} that knows how to put some basic
 * stats about itself into the dashboard.
 */
public class NamedMotor {

    private final String name;
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final String positionKey;
    private final String velocityKey;

    public NamedMotor(String name, int port) {

        this.name = name;
        this.motor = new CANSparkMax(port, MotorType.kBrushless);
        this.encoder = motor.getEncoder();

        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(false);
        motor.setOpenLoopRampRate(0.5);
        motor.setClosedLoopRampRate(0.5);

        this.positionKey = name + " Position";
        this.velocityKey = name + " Velocity";
    }

    public String getName() {
        return name;
    }

    public CANSparkMax getMotor() {
        return motor;
    }

    public RelativeEncoder getEncoder() {
        return encoder;
    }

    public double getPosition() {
        return encoder.getPosition();
    }
    
    public void set(double speed) {
        motor.set(speed);
    }

    /**
     * Halts the motor (i.e. brings it to a stop) by putting it in "brake"
     * mode and then applying a 0 duty cycle (basically saying "turn it off")
     */
    public void halt() {
        getMotor().setIdleMode(IdleMode.kBrake);
        getMotor().getPIDController().setReference(0.0, ControlType.kDutyCycle);
    }

    public void updateDashboard() {
        SmartDashboard.putNumber(positionKey, encoder.getPosition());
        SmartDashboard.putNumber(velocityKey, encoder.getVelocity());
        SmartDashboard.putNumber(name + " VCF", encoder.getVelocityConversionFactor());
        SmartDashboard.putNumber(name + " PCF", encoder.getPositionConversionFactor());
    }
}
