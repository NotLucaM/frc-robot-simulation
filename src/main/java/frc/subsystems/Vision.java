package frc.subsystems;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.apriltag.AprilTagFieldLayout;
import frc.robot.Commands;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.PhotonUtils;

import static frc.constants.FieldConstants.CAMERA_TO_ROBOT;
import static frc.constants.FieldConstants.*;
import static frc.constants.VisionConstants.*;

public class Vision extends SubsystemBase {

    public enum State {
        DRIVE, TARGETING
    }

    private final PhotonCamera camera = new PhotonCamera("tape");
    private final Logger log = Logger.getInstance();

    private State currentState = State.DRIVE;
    private VisionLEDMode wantedLEDs;
    private int wantedIndex = 0;
    private boolean driverMode = true;
    private boolean wantedChange = false;
    private boolean wantedSnapshot;

    @Override
    public void update(Commands commands, RobotState state) {
        switch (commands.visionWantedLEDs) {
            case DRIVE:
                wantedLEDs = VisionLEDMode.kOff;
                wantedIndex = 0;
                driverMode = true;

                if (currentState != State.DRIVE) {
                    wantedChange = true;
                    currentState = State.DRIVE;
                }
                break;
            case TARGETING:
                wantedLEDs = VisionLEDMode.kOn;
                wantedIndex = 0;
                driverMode = false;

                if (currentState != State.TARGETING) {
                    wantedChange = true;
                    currentState = State.TARGETING;
                }
                break;
        }
        wantedSnapshot = commands.visionWantedSnapshot;
    }

    @Override
    public void write() {
        if (wantedSnapshot) camera.takeOutputSnapshot();

        if (wantedChange) {
            camera.setPipelineIndex(wantedIndex);
            camera.setLED(wantedLEDs);
            camera.setDriverMode(driverMode);
            wantedChange = false;
        }
    }

    @Override
    public void read(RobotState state) {
        PhotonPipelineResult result = camera.getLatestResult();
        state.visionLatency = result.getLatencyMillis();

        if (result.hasTargets()) {
            state.visionTarget = result.getBestTarget();
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
                    state.driveYaw, TARGET_POSE, CAMERA_TO_ROBOT);
            log.recordOutput("Vision/estimatedFieldPos", fieldPos);

            // TODO: ensure time is correct
            state.drivePose.addVisionMeasurement(fieldPos, result.getTimestampSeconds());
        } else {
            state.visionTarget = null;
            state.visionDistanceToTarget = 0.0;
        }

        var aprilTags = result.getTargets();
        for (var tag : aprilTags) {
            int targetId = tag.getFiducialId();
            double poseAmbiguity = tag.getPoseAmbiguity();

            if (APRIL_FIELD.getTagPose(targetId).isEmpty()) {
                break;
            }
            Pose3d targetFieldPose = APRIL_FIELD.getTagPose(targetId).get();

            Transform3d bestCameraToTarget = tag.getBestCameraToTarget();
            Transform3d alternateCameraToTarget = tag.getAlternateCameraToTarget();

            // TODO: ensure adding is correct
            Pose3d bestRobotField = targetFieldPose.plus(bestCameraToTarget);
            Pose3d altRobotField = targetFieldPose.plus(alternateCameraToTarget);

            Pose2d currentLocation = state.drivePose.getEstimatedPosition();
            Translation2d currentTranslation2d = currentLocation.getTranslation();
            Translation3d currentTranslation = new Translation3d(currentTranslation2d.getX(),
                    currentTranslation2d.getY(), CAMERA_HEIGHT_METERS);
            var bestDist = bestRobotField.getTranslation().getDistance(currentTranslation);
            var altDist = altRobotField.getTranslation().getDistance(currentTranslation);

            // TODO: ensure time is correct
            if (bestDist <= altDist || poseAmbiguity < 0.2) {
                state.drivePose.addVisionMeasurement(bestRobotField.toPose2d(), result.getTimestampSeconds());
            } else {
                state.drivePose.addVisionMeasurement(altRobotField.toPose2d(), result.getTimestampSeconds());
            }
        }
    }

    @Override
    public void configure() {
        camera.setPipelineIndex(DEFAULT_INDEX);
    }
}
