package frc.constants;

import edu.wpi.first.math.util.Units;

public class VisionConstants {
    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(44.25),
                                TARGET_HEIGHT_METERS = Units.inchesToMeters(98.19) - Units.inchesToMeters(81.19),
                                CAMERA_PITCH_RADIANS = Units.degreesToRadians(0.0);

    public static final int DEFAULT_INDEX = 0;
}
