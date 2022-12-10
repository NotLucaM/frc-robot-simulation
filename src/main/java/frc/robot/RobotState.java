package frc.robot;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

public class RobotState {

    /* DRIVE */
    public DifferentialDrivePoseEstimator drivePose = new DifferentialDrivePoseEstimator(new Rotation2d(), new Pose2d(),
            new MatBuilder<>(Nat.N5(), Nat.N1()).fill(0.02, 0.02, 0.01, 0.02, 0.02), // State measurement standard deviations. X, Y, theta.
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), // Local measurement standard deviations. Left encoder, right encoder, gyro.
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01));
    public boolean gyroReady = false;
    public Rotation2d driveYaw;
    public double driveYawAngularVelocityDegrees = 0.0;
    public DifferentialDriveWheelSpeeds driveWheelSpeeds = new DifferentialDriveWheelSpeeds(0.0, 0.0);
    public double driveLeftPosition, driveRightPosition;
    public boolean driveIsQuickTurning, driveIsSlowTurning, driveIsSlowMoving;

    /* VISION */
    public double visionLatency = 0.0;
    public double visionDistanceToTarget = 0.0;
    public PhotonTrackedTarget visionTarget = null;

    public void log() {
        Logger log = Logger.getInstance();
        log.recordOutput("Vision/latency", visionLatency);
        log.recordOutput("Vision/distanceToTarget", visionDistanceToTarget);
        log.recordOutput("Vision/visionTarget", visionTarget != null);

        log.recordOutput("Drive/gyroReady", gyroReady);
        log.recordOutput("Drive/driveYaw", driveYaw.getDegrees());
        log.recordOutput("Drive/driveAngularVelocity", driveYawAngularVelocityDegrees);
        log.recordOutput("Drive/driveWheelSpeedLeft", driveWheelSpeeds.leftMetersPerSecond);
        log.recordOutput("Drive/driveWheelSpeedRight", driveWheelSpeeds.rightMetersPerSecond);
        log.recordOutput("Drive/driveLeftPosition", driveLeftPosition);
        log.recordOutput("Drive/driveRightPosition", driveRightPosition);
        log.recordOutput("Drive/position", drivePose.getEstimatedPosition());
    }
}
