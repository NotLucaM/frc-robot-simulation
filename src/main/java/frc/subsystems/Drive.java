package frc.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import frc.robot.Commands;
import frc.robot.RobotState;
import frc.subsystems.controllers.drive.ChezyDriveController;
import frc.util.DriveOutputs;
import frc.util.control.Falcon;
import frc.util.control.Pigeon;

import java.util.function.Function;

import static frc.constants.DriveConstants.*;
import static frc.constants.FieldConstants.*;
import static frc.constants.PortConstants.*;

public class Drive extends SubsystemBase {

    public enum State {
        NEUTRAL, TELEOP, OUTPUTS, FOLLOW_PATH, TURN
    }

    public abstract static class DriveController {

        protected DriveOutputs outputs = new DriveOutputs();

        public final DriveOutputs update(Commands commands, RobotState state) {
            updateSignal(commands, state);
            return outputs;
        }

        public abstract void updateSignal(Commands commands, RobotState state);
    }

    private final Falcon leftMasterFalcon = new Falcon(DRIVE_LEFT_MASTER_ID, "Drive Left Master"),
            leftSlaveFalcon = new Falcon(DRIVE_LEFT_SLAVE_ID, "Drive Left Slave");
    private final Falcon rightMasterFalcon = new Falcon(DRIVE_RIGHT_MASTER_ID, "Drive Right Master"),
            rightSlaveFalcon = new Falcon(DRIVE_RIGHT_SLAVE_ID, "Drive Right Slave");
    private final Pigeon gyro = new Pigeon(9);

    private final DifferentialDrivetrainSim driveSim = new DifferentialDrivetrainSim(
            DCMotor.getFalcon500(2),
            DRIVE_GEAR_RATIO,
            ROBOT_MOI,
            ROBOT_MASS,
            ROBOT_WHEEL_RADIUS,
            WHEEL_DISTANCE,
            null
    );

    private final double[] gyroAngles = new double[3], gyroAngularVelocities = new double[3];

    private State driveState = State.NEUTRAL;
    private DriveController controller;
    private DriveOutputs outputs = new DriveOutputs();

    @Override
    public void update(Commands commands, RobotState state) {
        State wantedState = commands.getDriveWantedState();
        boolean isNewState = driveState != wantedState;
        driveState = wantedState;

        if (isNewState) {
            switch (wantedState) {
                case NEUTRAL -> controller = null;
                case TELEOP -> controller = new ChezyDriveController();
            }
        }
        if (controller == null) {
            outputs.leftOutput.setIdle();
            outputs.rightOutput.setIdle();
        } else {
            outputs = controller.update(commands, state);
        }
    }

    @Override
    public void write() {
        log.recordOutput("Drive/leftReference", outputs.leftOutput.getReference());
        log.recordOutput("Drive/rightReference", outputs.leftOutput.getReference());
        leftMasterFalcon.setOutput(outputs.leftOutput);
        rightMasterFalcon.setOutput(outputs.rightOutput);
        log.recordOutput("Simulation/motorOutputPO", leftMasterFalcon.getMotorOutputPercent());
        log.recordOutput("Simulation/motorVolts", leftMasterFalcon.getBusVoltage());
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

        // don't update position if gyro is not working
        if (state.gyroReady) {
            state.drivePose.update(state.driveYaw, state.driveWheelSpeeds,
                    state.driveLeftPosition, state.driveRightPosition);
        } else {
            // TODO: update position without gyro
        }
    }

    @Override
    public void simulate() {
        var leftSim = leftMasterFalcon.getSimCollection();
        var rightSim = rightMasterFalcon.getSimCollection();
        var pigeonSim = gyro.getSimCollection();

        leftSim.setBusVoltage(RobotController.getBatteryVoltage());
        rightSim.setBusVoltage(RobotController.getBatteryVoltage());

        driveSim.setInputs(leftSim.getMotorOutputLeadVoltage(),
                            rightSim.getMotorOutputLeadVoltage());

        driveSim.update(0.02);

        Function<Double, Integer> distNative = (Double distance) -> (int) (distance / POSITION_CONVERSION);

        Function<Double, Integer> velNative = (Double velocity) -> (int) (velocity / VELOCITY_CONVERSION);

        leftSim.setIntegratedSensorRawPosition(distNative.apply(driveSim.getLeftPositionMeters()));
        rightSim.setIntegratedSensorRawPosition(distNative.apply(driveSim.getRightPositionMeters()));

        leftSim.setIntegratedSensorVelocity(velNative.apply(driveSim.getLeftVelocityMetersPerSecond()));
        rightSim.setIntegratedSensorVelocity(velNative.apply(driveSim.getRightVelocityMetersPerSecond()));

        pigeonSim.setRawHeading(driveSim.getHeading().getDegrees());
    }

    @Override
    public void configure() {
        leftMasterFalcon.configAllSettings(DRIVE_LEFT_MASTER_CONFIG);
        leftSlaveFalcon.configAllSettings(DRIVE_LEFT_SLAVE_CONFIG);
        leftMasterFalcon.setConversionFactors(VELOCITY_CONVERSION, POSITION_CONVERSION);
        leftSlaveFalcon.follow(leftMasterFalcon);

        rightMasterFalcon.configAllSettings(DRIVE_RIGHT_MASTER_CONFIG);
        rightSlaveFalcon.configAllSettings(DRIVE_RIGHT_SLAVE_CONFIG);
        rightMasterFalcon.setConversionFactors(VELOCITY_CONVERSION, POSITION_CONVERSION);
        rightSlaveFalcon.follow(rightMasterFalcon);

        rightMasterFalcon.setSelectedSensorPosition(0.0);
        leftMasterFalcon.setSelectedSensorPosition(0.0);
    }
}
