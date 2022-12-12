package frc.constants;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

import java.util.List;

public class FieldConstants {

    public static final Pose2d TARGET_POSE = new Pose2d();
    public static final Transform2d CAMERA_TO_ROBOT = new Transform2d();
    public static final double FIELD_LENGTH = 0.0,
                                FIELD_WIDTH = 0.0;
    public static final double ROBOT_MOI = 8.0;
    public static final double ROBOT_MASS = 60;
    public static final double ROBOT_WHEEL_RADIUS = Units.inchesToMeters(4);
    public static final double WHEEL_DISTANCE = 0.1;
    public static final double TRACK_WIDTH = Units.inchesToMeters(32);

    public static List<AprilTag> APRIL_TAGS = List.of(
            // Upper Hub Blue-Near
            new AprilTag(3,
            new Pose3d(
                    Units.inchesToMeters(332.321),
                    Units.inchesToMeters(183.676),
                    Units.inchesToMeters(95.186),
                    new Rotation3d(0, Math.toRadians(26.75), Math.toRadians(69)))),

            // Blue Terminal Near Station
            new AprilTag(1,
            new Pose3d(
                    Units.inchesToMeters(4.768),
                    Units.inchesToMeters(67.631),
                    Units.inchesToMeters(35.063),
                    new Rotation3d(0, 0, Math.toRadians(46.25)))
            ),

            // Blue Hangar Truss - Hub
            new AprilTag(2,
            new Pose3d(
                    Units.inchesToMeters(127.272),
                    Units.inchesToMeters(216.01),
                    Units.inchesToMeters(67.932),
                    new Rotation3d(0, 0, 0))
            )
    );
    public static final AprilTagFieldLayout APRIL_FIELD = new AprilTagFieldLayout(APRIL_TAGS, FIELD_LENGTH, FIELD_WIDTH);
}
