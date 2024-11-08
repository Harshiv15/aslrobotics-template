package frc.robot.subsystems.feeder;

public class FeederIOSim implements FeederIO {
  @Override
  public void updateInputs(FeederIOInputs inputs) {}

  @Override
  public void setVoltage(double volts) {}

  @Override
  public void setVelocity(double radiansPerSecond, double ffVolts) {}

  @Override
  public void configurePID(double kP, double kI, double kD) {}
}
