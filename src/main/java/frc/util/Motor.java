package frc.util;

public interface Motor {

    public void setOutput(ControllerOutput output);

    public void setConversionFactors(double velocityConversion, double positionConversion);
    public double getConvertedVelocity();
    public double getConvertedPosition();
}
