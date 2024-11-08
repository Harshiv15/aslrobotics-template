package frc.robot.subsystems.pivot;

import com.revrobotics.*;

public class PivotIOReal implements PivotIO {
  private final CANSparkMax pivot =
      new CANSparkMax(PivotMap.LEFT_PIVOT_ID, CANSparkLowLevel.MotorType.kBrushless);
  private final RelativeEncoder encoder = pivot.getEncoder();
  private final SparkPIDController pid = pivot.getPIDController();

  public PivotIOReal() {
    pivot.restoreFactoryDefaults();

    pivot.setCANTimeout(250);

    pivot.setInverted(false);

    pivot.enableVoltageCompensation(12.0);
    pivot.setSmartCurrentLimit(30);

    pivot.burnFlash();
  }

  @Override
  public void updateInputs(PivotIO.PivotIOInputs inputs) {
    //    inputs.climberLeftPositionMeters =    TODO: Make inputs
    //    inputs.climberRightPositionMeters =
  }

  @Override
  public void setPosition(double climberPositionRad, double ffVolts) {
    pid.setReference(
        climberPositionRad,
        CANSparkBase.ControlType.kPosition,
        0,
        ffVolts,
        SparkPIDController.ArbFFUnits.kVoltage);
  }

  @Override
  public void setVoltage(double volts) {
    pivot.setVoltage(volts);
  }

  public void setHoming(boolean homingBool) {} // TODO: Figure out wtf is set Homing

  @Override
  public void configurePID(double kP, double kI, double kD) {
    // Left PID Values
    pid.setP(kP, 0);
    pid.setI(kI, 0);
    pid.setD(kD, 0);
    pid.setFF(0, 0);
  }
}
