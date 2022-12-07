package frc.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Commands;
import frc.robot.State;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.PhotonUtils;

import static frc.constants.FieldConstants.CAMERA_TO_ROBOT;
import static frc.constants.FieldConstants.*;
import static frc.constants.VisionConstants.*;

public class Vision extends SubsystemBase {

    private final PhotonCamera aprilCamera = new PhotonCamera("april");
    private final PhotonCamera tapeCamera = new PhotonCamera("tape");

    private VisionLEDMode wantedLEDs;
    private boolean wantedSnapshot;

    @Override
    public void update(Commands commands, State state) {
        wantedLEDs = commands.visionWantedLEDs;
        wantedSnapshot = commands.visionWantedSnapshot;
    }

    @Override
    public void write() {
        if (wantedSnapshot) tapeCamera.takeOutputSnapshot();
        if (tapeCamera.getLEDMode() != wantedLEDs) tapeCamera.setLED(wantedLEDs);
    }

    @Override
    public void read(State state) {
        PhotonPipelineResult tapeResult = tapeCamera.getLatestResult();
        state.visionLatency = tapeResult.getLatencyMillis();

        if (tapeResult.hasTargets()) {
            state.visionTarget = tapeResult.getBestTarget();
            state.visionDistanceToTarget =
                    PhotonUtils.calculateDistanceToTargetMeters(
                            CAMERA_HEIGHT_METERS,
                            TARGET_HEIGHT_METERS,
                            CAMERA_PITCH_RADIANS,
                            Units.degreesToRadians(state.visionTarget.getPitch()));

            var fieldPos = PhotonUtils.estimateFieldToRobot(
                    CAMERA_HEIGHT_METERS, TARGET_HEIGHT_METERS, CAMERA_PITCH_RADIANS,
                    0.0,
                    Rotation2d.fromDegrees(-state.visionTarget.getYaw()),
                    state.driveRotation, TARGET_POSE, CAMERA_TO_ROBOT);

            // TODO: ensure time is correct
            state.drivePose.addVisionMeasurement(fieldPos, tapeResult.getTimestampSeconds());
        } else {
            state.visionTarget = null;
            state.visionDistanceToTarget = null;
        }

        PhotonPipelineResult aprilResult = aprilCamera.getLatestResult();
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
        aprilCamera.setPipelineIndex(DEFAULT_INDEX);
        tapeCamera.setPipelineIndex(DEFAULT_INDEX);
    }
}
