package frc.robot.lib;

import com.choreo.lib.ChoreoTrajectoryState;
import com.google.gson.Gson;
import com.google.gson.JsonObject;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.Arrays;
import java.util.List;

public class ChoreoTrajectory {
    private static final Gson gson = new Gson();
    private com.choreo.lib.ChoreoTrajectory path;
    private final List<EventMarker> eventMarkers;

    public ChoreoTrajectory(List<ChoreoTrajectoryState> samples, List<EventMarker> eventMarkers) {
        this.path = new com.choreo.lib.ChoreoTrajectory(samples);
        this.eventMarkers = eventMarkers;
    }

    public ChoreoTrajectory(com.choreo.lib.ChoreoTrajectory path, List<EventMarker> eventMarkers) {
        this.path = path;
        this.eventMarkers = eventMarkers;
    }

    public ChoreoTrajectory flipped() {
        this.path = this.path.flipped();
        return this;
    }

    public List<EventMarker> getEventMarkers() {
        return eventMarkers;
    }

    public static ChoreoTrajectory getTrajectory(String trajName) {
        var traj_dir = new File(Filesystem.getDeployDirectory(), "choreo");
        var traj_file = new File(traj_dir, trajName + ".traj");

        try {
            BufferedReader reader = null;
            reader = new BufferedReader(new FileReader(traj_file));

            JsonObject wholeTrajectory = gson.fromJson(reader, JsonObject.class);

            com.choreo.lib.ChoreoTrajectory path = gson.fromJson(wholeTrajectory, com.choreo.lib.ChoreoTrajectory.class);
            List<EventMarker> eventMarkers = Arrays.asList(gson.fromJson(wholeTrajectory.get("eventMarkers"), EventMarker[].class));
            return new ChoreoTrajectory(path,eventMarkers);
        } catch (FileNotFoundException ex) {
            System.out.println(ex.getMessage());
            System.out.println(ex.getStackTrace());
            DriverStation.reportError(ex.getMessage(), ex.getStackTrace());
        }

        return null;
    }

    public double getTotalTime() {
        return this.path.getTotalTime();
    }

    public ChoreoTrajectoryState sample(double timestamp) {
        return this.path.sample(timestamp);
    }

    public ChoreoTrajectoryState getFinalState() {
        return this.path.getFinalState();
    }

    public Pose2d getInitialPose() {
        return this.path.getInitialPose();
    }

    public Pose2d[] getPoses() {
        return this.path.getPoses();
    }

    public Pose2d getFinalPose() {
        return this.path.getFinalPose();
    }
}
