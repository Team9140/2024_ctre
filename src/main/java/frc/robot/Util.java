package frc.robot;

public class Util {
    private final static double deadband = 0.12;

    public static double applyDeadband(double in) {
        if (Math.abs(in) < deadband) {
            return 0.0;
        } else if (in > 0) {
            return (in - deadband) / (1.0 - deadband);
        } else {
            return (in + deadband) / (1.0 - deadband);
        }
    }
}
