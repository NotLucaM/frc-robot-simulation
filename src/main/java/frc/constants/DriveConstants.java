package frc.constants;

import com.ctre.phoenix.motorcontrol.can.BaseTalonPIDSetConfiguration;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public class DriveConstants {

    public static final TalonFXConfiguration DRIVE_LEFT_MASTER_CONFIG = new TalonFXConfiguration();
    public static final TalonFXConfiguration DRIVE_LEFT_SLAVE_CONFIG = new TalonFXConfiguration();
    public static final TalonFXConfiguration DRIVE_RIGHT_MASTER_CONFIG = new TalonFXConfiguration();
    public static final TalonFXConfiguration DRIVE_RIGHT_SLAVE_CONFIG = new TalonFXConfiguration();

    static {
        var driveLeftMasterPID = new SlotConfiguration();
        driveLeftMasterPID.kF = 0.0;
        driveLeftMasterPID.kP = 0.0;
        DRIVE_LEFT_MASTER_CONFIG.slot0 = driveLeftMasterPID;

        var driveLeftSlavePID = new SlotConfiguration();
        driveLeftSlavePID.kF = 0.0;
        driveLeftSlavePID.kP = 0.0;
        DRIVE_LEFT_SLAVE_CONFIG.slot0 = driveLeftSlavePID;

        var driveRightMasterPID = new SlotConfiguration();
        driveRightMasterPID.kF = 0.0;
        driveRightMasterPID.kP = 0.0;
        DRIVE_RIGHT_MASTER_CONFIG.slot0 = driveRightMasterPID;

        var driveRightSlavePID = new SlotConfiguration();
        driveRightSlavePID.kF = 0.0;
        driveRightSlavePID.kP = 0.0;
        DRIVE_RIGHT_SLAVE_CONFIG.slot0 = driveRightSlavePID;
    }
}
