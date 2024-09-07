package frc.robot.subsystems;

import java.util.EnumSet;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTable.TableEventListener;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;

public class LimeLight extends SubsystemBase {

    public static final LimeLight front = new LimeLight("limelight-front");

    private String name;
    private int priorityID;

    private LimeLight(String nm) {
        this.name = nm;
    }

    public static class VisionResult {
        public double tx;
        public double ty;
        public double timestamp;
        public boolean valid;
    }

    private volatile VisionResult latestResult = new VisionResult();

    public VisionResult getLatest() {
        return this.latestResult;
    }

    public void setPriorityTag(int id) {
        this.priorityID = id;
    }

    public void setRobotOrientation(Rotation2d direction) {
        LimelightHelpers.SetRobotOrientation(
                this.name, direction.getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    private class Listener implements TableEventListener {

        public void accept(NetworkTable table, String key, NetworkTableEvent event) {
            if (key.equals("json")) {
                double before = Timer.getFPGATimestamp();
                LimelightResults llResult = LimelightHelpers.getLatestResults(LimeLight.this.name);
                VisionResult vr = new VisionResult();

                vr.timestamp = before - llResult.latency_pipeline / 1000.0 - llResult.latency_capture / 1000.0;

                for (LimelightTarget_Fiducial fiducial : llResult.targets_Fiducials) {
                    if (fiducial.fiducialID == LimeLight.this.priorityID) {
                        vr.tx = fiducial.tx;
                        vr.ty = fiducial.ty;
                        vr.valid = true;
                    }
                }

                latestResult = vr;
            }
        }
    }

    private int m_listenerID = -1;

    public synchronized void start() {
        if (m_listenerID < 0) {
            m_listenerID = NetworkTableInstance.getDefault().getTable(this.name).addListener("json",
                    EnumSet.of(Kind.kValueAll), new Listener());
        }
    }
}
