package frc.robot.subsystems;

import java.util.EnumSet;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTable.TableEventListener;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;

public class LimeLight extends SubsystemBase {

    private static LimeLight instance;

    public static LimeLight getInstance() {
        if (instance == null) {
            return instance = new LimeLight();
        } else {
            return instance;
        }
    }

    public static class VisionResult {
        public double tx;
        public double ty;
        public double timestamp;
        public boolean valid;
    }

    private volatile VisionResult latestResult = null;

    public VisionResult getLatest() {
        return this.latestResult;
    }

    private class Listener implements TableEventListener {
        public void accept(NetworkTable table, String key, NetworkTableEvent event) {
            if (key.equals("json")) {
                double before = Timer.getFPGATimestamp();
                LimelightResults llResult = LimelightHelpers.getLatestResults("limelight-front");
                VisionResult vr = new VisionResult();

                vr.timestamp = before - llResult.latency_pipeline / 1000.0 - llResult.latency_capture / 1000.0;

                if (llResult.targets_Fiducials.length > 0) {
                    vr.tx = llResult.targets_Fiducials[0].tx;
                    vr.ty = llResult.targets_Fiducials[0].ty;
                    vr.valid = llResult.valid;
                } else {
                    vr.tx = 0;
                    vr.ty = 0;
                    vr.valid = false;
                }

                latestResult = vr;

            }
        }
    }

    private int m_listenerID = -1;

    public synchronized void start() {
        if (m_listenerID < 0) {
            m_listenerID = NetworkTableInstance.getDefault().getTable("limelight-front").addListener("json",
                    EnumSet.of(Kind.kValueAll), new Listener());
        }
    }
}
