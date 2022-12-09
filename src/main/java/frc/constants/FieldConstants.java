package frc.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.apriltag.AprilTag;
import edu.wpi.first.wpilibj.apriltag.AprilTagFieldLayout;

import java.util.List;

public class FieldConstants {

    public static final Pose2d TARGET_POSE = new Pose2d();
    public static final Transform2d CAMERA_TO_ROBOT = new Transform2d();
    public static final double FIELD_LENGTH = 0.0,
                                FIELD_WIDTH = 0.0;

    public static List<AprilTag> APRIL_TAGS = List.of(
            new AprilTag(0, new Pose3d())
    );
    public static final AprilTagFieldLayout APRIL_FIELD = new AprilTagFieldLayout(APRIL_TAGS, FIELD_LENGTH, FIELD_WIDTH);
}
