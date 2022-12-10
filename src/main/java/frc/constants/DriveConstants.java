package frc.constants;

import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.math.util.Units;

import static frc.constants.FieldConstants.ROBOT_WHEEL_RADIUS;

public class DriveConstants {

    public static final TalonFXConfiguration DRIVE_LEFT_MASTER_CONFIG = new TalonFXConfiguration();
    public static final TalonFXConfiguration DRIVE_LEFT_SLAVE_CONFIG = new TalonFXConfiguration();
    public static final TalonFXConfiguration DRIVE_RIGHT_MASTER_CONFIG = new TalonFXConfiguration();
    public static final TalonFXConfiguration DRIVE_RIGHT_SLAVE_CONFIG = new TalonFXConfiguration();

    public static final double DRIVE_GEAR_RATIO = 10.0;

    public static final double POSITION_CONVERSION = (1.0 / 2048.0) * (1.0 / DRIVE_GEAR_RATIO) * ROBOT_WHEEL_RADIUS * Math.PI,
            VELOCITY_CONVERSION = POSITION_CONVERSION * 10;

    public static final double DEAD_BAND = 0.01;

    public static final double TURN_SENSITIVITY = 0.65,
            SLOW_TURN_SCALAR = 0.1,
            QUICK_TURN_SCALAR = 0.4,
            QUICK_STOP_SCALAR = 0.0,
            QUICK_STOP_DEADBAND = 0.5,
            QUICK_STOP_WEIGHT = 0.125,
            LOW_NEGATIVE_INERTIA_CLOSE_SCALAR = 3.0,
            LOW_NEGATIVE_INERTIA_FAR_SCALAR = 5.0,
            LOW_NEGATIVE_INERTIA_THRESHOLD = 0.65,
            LOW_NEGATIVE_INERTIA_TURN_SCALAR = 2.5,
            SLOW_MOVE_SCALAR = 0.1,
            WHEEL_NON_LINEARITY = 0.5;
    public static final int NONLINEAR_PASSES = 3;

    static {
        var driveLeftMasterPID = new SlotConfiguration();
        driveLeftMasterPID.kF = 0.04697616751;
        driveLeftMasterPID.kP = 0.015;
        DRIVE_LEFT_MASTER_CONFIG.slot0 = driveLeftMasterPID;

        var driveRightMasterPID = new SlotConfiguration();
        driveRightMasterPID.kF = 0.04697616751;
        driveRightMasterPID.kP = 0.015;
        DRIVE_RIGHT_MASTER_CONFIG.slot0 = driveRightMasterPID;
    }
}
