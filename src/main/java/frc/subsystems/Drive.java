package frc.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import frc.robot.Commands;
import frc.robot.RobotState;
import frc.util.Falcon;

import static frc.constants.DriveConstants.*;
import static frc.constants.PortConstants.*;

public class Drive extends SubsystemBase {

    public enum State {
        NEUTRAL, TELEOP, OUTPUTS, FOLLOW_PATH, TURN
    }

    private final Falcon leftMasterFalcon = new Falcon(DRIVE_LEFT_MASTER_ID, "Drive Left Master"),
            leftSlaveFalcon = new Falcon(DRIVE_LEFT_SLAVE_ID, "Drive Left Slave");
    private final Falcon rightMasterFalcon = new Falcon(DRIVE_RIGHT_MASTER_ID, "Drive Right Master"),
            rightSlaveFalcon = new Falcon(DRIVE_RIGHT_SLAVE_ID, "Drive Right Slave");
    private final PigeonIMU gyro = new PigeonIMU(0);
    private final double[] gyroAngles = new double[3], gyroAngularVelocities = new double[3];

    @Override
    public void update(Commands commands, RobotState state) {

    }

    @Override
    public void write() {

    }

    @Override
    public void read(RobotState state) {
        var gyroState = gyro.getState();
        if (gyroState == PigeonIMU.PigeonState.Ready) {
            state.gyroReady = true;
            gyro.getYawPitchRoll(gyroAngles);
            state.driveYaw = Rotation2d.fromDegrees(gyroAngles[0]);
            gyro.getRawGyro(gyroAngularVelocities);
            state.driveYawAngularVelocityDegrees = gyroAngularVelocities[2];
        } else {
            state.gyroReady = false;
        }

        state.driveWheelSpeeds.leftMetersPerSecond = leftMasterFalcon.getConvertedVelocity();
        state.driveWheelSpeeds.rightMetersPerSecond = rightMasterFalcon.getConvertedVelocity();
        state.driveLeftPosition = leftMasterFalcon.getConvertedPosition();
        state.driveRightPosition = rightMasterFalcon.getConvertedPosition();

        state.drivePose.update(state.driveYaw, state.driveWheelSpeeds,
                state.driveLeftPosition, state.driveRightPosition);
    }

    @Override
    public void configure() {
        leftMasterFalcon.configAllSettings(DRIVE_LEFT_MASTER_CONFIG);
        leftSlaveFalcon.configAllSettings(DRIVE_LEFT_SLAVE_CONFIG);
        leftSlaveFalcon.follow(leftMasterFalcon);

        rightMasterFalcon.configAllSettings(DRIVE_RIGHT_MASTER_CONFIG);
        rightSlaveFalcon.configAllSettings(DRIVE_RIGHT_SLAVE_CONFIG);
        rightSlaveFalcon.follow(rightMasterFalcon);
    }
}
