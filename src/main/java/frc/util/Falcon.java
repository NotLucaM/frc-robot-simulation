package frc.util;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Falcon extends TalonFX {

    private final String name;

    private double velocityConversion = 1.0;
    private double positionConversion = 1.0;

    public Falcon(int deviceNumber, String name, String canbus) {
        super(deviceNumber, canbus);
        this.name = name;
    }

    public Falcon(int deviceNumber, String name) {
        super(deviceNumber);
        this.name = name;
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
