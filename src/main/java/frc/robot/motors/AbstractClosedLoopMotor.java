package frc.robot.motors;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.util.PIDConstant;

/**
 * Base class for "closed loop" motors - these will have their power level managed indirectly 
 * via a PID controller rather than directly. This class takes care of reading the PID algorithm
 * parameters from the SmartDashboard for tuning.
 * 
 * Here's some basic information about PID theory - https://www.ni.com/en-us/innovations/white-papers/06/pid-theory-explained.html
 */
public abstract class AbstractClosedLoopMotor extends NamedMotor {

    private final SparkMaxPIDController controller;
    private PIDConstant constants;

    public AbstractClosedLoopMotor(String name, int port, PIDConstant constants) {
        super(name, port);
        this.controller = getMotor().getPIDController();
        setConstants(constants);
    }

    public SparkMaxPIDController getController() {
        return controller;
    }

    public PIDConstant getConstants() {
        return constants;
    }

    private void setConstants(PIDConstant constants) {
        this.constants = constants;
        constants.configPID(controller);
    }

    /**
     * Lets the motor coast by putting it in "coast" mode and then applying a
     * 0 duty cycle (basically saying "turn it off")
     */
    public void coast() {
        getMotor().setIdleMode(IdleMode.kCoast);
        getController().setReference(0.0, ControlType.kDutyCycle);
    }
}
