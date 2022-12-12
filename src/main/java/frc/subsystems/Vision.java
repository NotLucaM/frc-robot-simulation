package frc.subsystems;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.constants.FieldConstants;
import frc.robot.Commands;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;
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

    SimVisionSystem simVision =
            new SimVisionSystem(
                    "tape",
                    120.0,
                    new Transform3d(
                            new Translation3d(0.0, 0.0, CAMERA_HEIGHT_METERS),
                            new Rotation3d(0.0, CAMERA_PITCH_RADIANS, 0.0)),
                    20,
                    640,
                    480,
                    10);

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

            if (APRIL_FIELD.getTagPose(targetId).isEmpty()) {
                break;
            }

            Pose3d targetFieldPose = APRIL_FIELD.getTagPose(targetId).get();
            log.recordOutput("Vision/targetFieldPose", targetFieldPose.toPose2d());
            Transform3d bestCameraToTarget = tag.getBestCameraToTarget();
            Pose3d bestRobotField = targetFieldPose.transformBy(bestCameraToTarget.inverse());

            System.out.println(bestRobotField);
            state.drivePose.addVisionMeasurement(bestRobotField.toPose2d(), result.getTimestampSeconds());
        }
    }

    @Override
    public void simulate(RobotState state) {
        simVision.processFrame(state.drivePoseSim.getPoseMeters());
    }

    @Override
    public void configure() {
        camera.setPipelineIndex(DEFAULT_INDEX);

        double targetWidth = Units.inchesToMeters(6); // meters
        double targetHeight = Units.inchesToMeters(6); // meters

        APRIL_TAGS.forEach(s -> simVision.addSimVisionTarget(new SimVisionTarget(s.pose, targetWidth, targetHeight, s.ID)));
    }
}
