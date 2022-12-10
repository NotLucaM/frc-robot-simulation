package frc.util.control;

public interface Motor {

    public void setOutput(ControllerOutput output);

    public void setConversionFactors(double velocityConversion, double positionConversion);
    public double getConvertedVelocity();
    public double getConvertedPosition();
}
