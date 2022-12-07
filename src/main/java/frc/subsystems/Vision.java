package frc.subsystems;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Commands;
import frc.robot.State;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.PhotonUtils;

import static frc.constants.VisionConstants.*;

public class Vision extends SubsystemBase {

    private PhotonCamera aprilCamera = new PhotonCamera("april");
    private PhotonCamera tapeCamera = new PhotonCamera("tape");
    private PhotonPipelineResult aprilResult;
    private PhotonPipelineResult tapeResult;

    @Override
    public void update(Commands commands, State state) {

    }

    @Override
    public void write() {

    }

    @Override
    public void read(State state) {
        tapeResult = tapeCamera.getLatestResult();
        state.visionLatency = tapeResult.getLatencyMillis();

        if (tapeResult.hasTargets()) {
            state.visionTarget = tapeResult.getBestTarget();
            state.visionDistanceToTarget =
                    PhotonUtils.calculateDistanceToTargetMeters(
                            CAMERA_HEIGHT_METERS,
                            TARGET_HEIGHT_METERS,
                            CAMERA_PITCH_RADIANS,
                            Units.degreesToRadians(state.visionTarget.getPitch()));
        } else {
            state.visionTarget = null;
            state.visionDistanceToTarget = null;
        }

        aprilResult = aprilCamera.getLatestResult();
        var aprilTags = aprilResult.getTargets();
        for (var tag : aprilTags) {
            int targetId = tag.getFiducialId();
            double poseAmbiguity = tag.getPoseAmbiguity();

            Transform3d bestCameraToTarget = tag.getBestCameraToTarget();
            Transform3d alternateCameraToTarget = tag.getAlternateCameraToTarget();
        }
    }

    @Override
    public void configure() {

    }
}
