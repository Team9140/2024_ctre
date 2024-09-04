package frc.robot.subsystems;

import java.util.Optional;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraVision extends SubsystemBase {

    public static final double MIN_TA_BACK = 0.15;
    public static final double MIN_TA_FRONT = 0.1;

    private static InterpolatingDoubleTreeMap mapping = new InterpolatingDoubleTreeMap();

    static {
        mapping.put(Double.valueOf(14.54), Double.valueOf(-0.7));
        mapping.put(Double.valueOf(9.54), Double.valueOf(-0.75));
        mapping.put(Double.valueOf(6.07), Double.valueOf(-0.8));
        mapping.put(Double.valueOf(3.09), Double.valueOf(-0.87));
        mapping.put(Double.valueOf(0.65), Double.valueOf(-0.9));
        mapping.put(Double.valueOf(-1.54), Double.valueOf(-0.94));
        mapping.put(Double.valueOf(-3.07), Double.valueOf(-0.97));
        mapping.put(Double.valueOf(-4.7), Double.valueOf(-0.99));
        mapping.put(Double.valueOf(-6.03), Double.valueOf(-1.01));
        mapping.put(Double.valueOf(-7.23), Double.valueOf(-1.03));
        mapping.put(Double.valueOf(-8.22), Double.valueOf(-1.045));
        mapping.put(Double.valueOf(-8.85), Double.valueOf(-1.047));
    }

    public static double getUnderhandAngle() {
        return mapping.get(Double.valueOf(LimelightHelpers.getTY("limelight-front")));
    }

    public static Optional<Double> backCamAngleToGoal() {
        if (LimelightHelpers.getTA("limelight-back") >= MIN_TA_BACK) {
            return Optional.of(LimelightHelpers.getTX("limelight-back"));
        } else {
            return Optional.empty();
        }
    }

    public static Optional<Double> frontCamAngleToGoal() {
        if (LimelightHelpers.getTA("limelight-front") >= MIN_TA_FRONT) {
            return Optional.of(LimelightHelpers.getTX("limelight-front"));
        } else {
            return Optional.empty();
        }
    }
}
