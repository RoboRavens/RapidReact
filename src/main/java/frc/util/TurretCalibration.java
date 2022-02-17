package frc.util;

public class TurretCalibration {
    
    public String name;
    public double kF;
    public double kP;
    public double kI;
    public double kD;
    public double target;

    /**
     * A "shot" object, storing all values needed for revving motors to the shot rpm with PID
     * @param name Name of the shot
     * @param kF F value for PID control
     * @param kP P value for PID control
     * @param kI I value for PID control
     * @param kD D value for PID control
     */
    public TurretCalibration(String name, double kF, double kP, double kI, double kD) {
        this.name = name;
        this.kF = kF;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
}
