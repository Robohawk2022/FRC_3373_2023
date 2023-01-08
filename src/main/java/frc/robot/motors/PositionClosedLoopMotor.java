package frc.robot.motors;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Subclass of {@link AbstractClosedLoopMotor} for motors that will be moved to
 * set positions, or rotated through set angles.
 * 
 * For these motors, you're extremely unlikely to "set them to 0" because that
 * doesn't mean to stop them, it means to unwind them some crazy number of
 * rotations back to a factory initial value.
 */
public class PositionClosedLoopMotor extends NamedMotor {

    private static final double DEFAULT_MAX_SPEED = 0.5;
    private static final double DEFAULT_THRESHOLD = 0.0005;
  
    private double maxSpeed;
    private double threshold;
    private double targetPosition;
    private double totalDelta;

    public PositionClosedLoopMotor(String name, int port) {
        super(name, port);
        getMotor().setIdleMode(IdleMode.kBrake);
        this.maxSpeed = DEFAULT_MAX_SPEED;
        this.threshold = DEFAULT_THRESHOLD;
    }

    public double getTargetPosition() {
      return targetPosition;
    }

    public void setMaxSpeed(double newMax) {
      maxSpeed = newMax;
    }

    public void setThreshold(double newThresh) {
      threshold = newThresh;
    }

    /**
     * Sets the target position to the current position
     */
    public void resetClosedLoopControl() {
        setTargetPosition(getPosition());
    }

    /**
     * Sets the target position for this motor, which also involves capturing
     * a "total delta" - how far away we are when this mehtod is called
     */
    public void setTargetPosition(double newTarget) {
        targetPosition = newTarget;
        totalDelta = targetPosition - getPosition();
    }

    /**
     * Sets the target position to the current position plus a number of rotationx
     */
    public void rotate(double rotations) {
      setTargetPosition(getPosition() + rotations);
    }

    /**
     * Called to update the speed as necessary to bring the current position
     * closer to the target position
     */
    public void updateSpeed() {
      
      double currentPosition = getPosition();
      double currentDelta = targetPosition - currentPosition;

      if (Math.abs(currentDelta) > threshold) {
        double proportionalSpeed = Math.abs(currentDelta/totalDelta) * maxSpeed;
        if (currentDelta < 0) {
          set(-proportionalSpeed);
        }
        else {
          set(proportionalSpeed);
        }
      }
      else {
        set(0);
      }
    }

    
    public void updateDashboard() {
        super.updateDashboard();
        SmartDashboard.putNumber(getName()+" Target Postion", targetPosition);
        SmartDashboard.putNumber(getName()+" Total Delta", totalDelta);
    }
}
