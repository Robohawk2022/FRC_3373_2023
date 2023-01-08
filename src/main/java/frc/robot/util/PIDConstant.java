package frc.robot.util;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;

public class PIDConstant {

    private final double kP, kI, kD, kIz, kFF, kMinOutput, kMaxOutput;

    public PIDConstant(double P, double I, double D, double FF, double IZone, double MINOutput,
            double MaxOutput) {
        this.kP=P;
        this.kI=I;
        this.kD=D;
        this.kFF=FF;
        this.kIz=IZone;
        this.kMinOutput=MINOutput;
        this.kMaxOutput=MaxOutput;
        
    }

    public void configPID(CANSparkMax motor) {
        SparkMaxPIDController pidcontrol = motor.getPIDController();
        pidcontrol.setP(kP);
        pidcontrol.setI(kI);
        pidcontrol.setD(kD);
        pidcontrol.setFF(kFF);
        pidcontrol.setIZone(kIz);
        pidcontrol.setOutputRange(kMinOutput, kMaxOutput);
    }

    public void configPID(SparkMaxPIDController pidcontrol) {
        pidcontrol.setP(kP);
        pidcontrol.setI(kI);
        pidcontrol.setD(kD);
        pidcontrol.setFF(kFF);

        pidcontrol.setIZone(kIz);
        pidcontrol.setOutputRange(kMinOutput, kMaxOutput);
    }

    public double getP() {
        return kP;
    }

    public double getI() {
        return kI;
    }

    public double getD() {
        return kD;
    }

    public double getIZone() {
        return kIz;
    }

    public double getFeedForward() {
        return kFF;
    }

    public double getMinOutput() {
        return kMinOutput;
    }

    public double getMaxOutput() {
        return kMaxOutput;
    }

    public String toString(){
        return "P="+kP+",I="+kI+",D="+kD+",FF="+kFF+",IZone="+kIz+",Range("+kMinOutput+","+kMaxOutput+")";
    }

}
