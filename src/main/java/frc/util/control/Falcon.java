package frc.util.control;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Falcon extends WPI_TalonFX implements Motor {

    private final String name;

    private double velocityConversion = 1.0;
    private double positionConversion = 1.0;

    public Falcon(int deviceNumber, String name, String canbus) {
        super(deviceNumber, canbus);
        this.name = name;
    }

    public Falcon(int deviceNumber, String name) {
        super(deviceNumber, "rio");
        this.name = name;
    }

    public void setOutput(ControllerOutput output) {
        if (output == null) {
            output = new ControllerOutput();
        }

        double reference = output.getReference();
        var controllerMode = switch (output.getControlMode()) {
            case PERCENT_OUTPUT -> TalonFXControlMode.PercentOutput;
            case POSITION -> TalonFXControlMode.Position;
            case VELOCITY -> TalonFXControlMode.Velocity;
            case PROFILED_POSITION -> TalonFXControlMode.MotionMagic;
            case PROFILED_VELOCITY -> TalonFXControlMode.MotionProfile;
        };
        double convertedReference = switch (output.getControlMode()) {
            case VELOCITY,PROFILED_VELOCITY -> reference / velocityConversion;
            case POSITION,PROFILED_POSITION -> reference / positionConversion;
            default -> reference;
        };
        set(controllerMode, convertedReference, DemandType.ArbitraryFeedForward, output.getArbitraryDemand());
    }

    public void setConversionFactors(double velocityConversion, double positionConversion) {
        this.velocityConversion = velocityConversion;
        this.positionConversion = positionConversion;
    }

    public double getConvertedVelocity() {
        return getSelectedSensorVelocity() * velocityConversion;
    }

    public double getConvertedPosition() {
        return getSelectedSensorPosition() * positionConversion;
    }
}
