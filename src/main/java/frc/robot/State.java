package frc.robot;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;

public class State {

    /* DRIVE */
    public DifferentialDrivePoseEstimator drivePose = new DifferentialDrivePoseEstimator(new Rotation2d(),
            0.0, 0.0,
            new Pose2d(),
            new MatBuilder<>(Nat.N5(), Nat.N1()).fill(0.02, 0.02, 0.01, 0.02, 0.02), // State measurement standard deviations. X, Y, theta.
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), // Local measurement standard deviations. Left encoder, right encoder, gyro.
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01));
    public Rotation2d driveRotation;

    /* VISION */
    public double visionLatency = 0.0;
    public Double visionDistanceToTarget = 0.0;
    public PhotonTrackedTarget visionTarget = null;
}
