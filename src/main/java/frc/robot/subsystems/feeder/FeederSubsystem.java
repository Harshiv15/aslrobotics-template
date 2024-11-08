package frc.robot.subsystems.feeder;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class FeederSubsystem extends SubsystemBase {

  public enum feederMode {
    OFF(0.0), // feeder is off
    FAST(12.0), // Maximum forward voltage
    REVERSE(-12.0); // Maximum reverse voltage

    final double voltage;

    feederMode(double voltage) {
      this.voltage = voltage;
    }
  }

  private final FeederIO io;
  private final FeederIO.FeederIOInputs inputs = new FeederIO.FeederIOInputs();
  private feederMode currentState = feederMode.OFF;

  // Debouncer to filter out noise or temporary spikes in current
  private final Debouncer currentDebouncer =
      new Debouncer(0.2, DebounceType.kRising); // 200ms debounce

  public FeederSubsystem(FeederIO io) {
    this.io = io;
    this.io.configurePID(1, 1, 1); // TODO: find PID Values
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    // Add logging or telemetry if needed
  }

  /** Set feeder to a specified mode using the enum */
  public void setfeederMode(feederMode mode) {
    currentState = mode;
    io.setVoltage(mode.voltage);
  }

  /** Stop the feeder */
  public Command stop() {
    return Commands.run(() -> setfeederMode(feederMode.OFF));
  }

  /** Run the feeder in reverse */
  public Command reverse() {
    return Commands.run(() -> setfeederMode(feederMode.REVERSE));
  }

  /** Run the feeder at maximum speed */
  public Command fast() {
    return Commands.run(() -> setfeederMode(feederMode.FAST));
  }

  /** Trigger based on current draw (beam brake alternative using current detection) */
  public Trigger hasNote() {
    return new Trigger(
        () ->
            this.currentState == feederMode.FAST // Detect only while feeder is running forward
                && currentDebouncer.calculate(
                    inputs.feederCurrentAmps > 40) // Debounced current detection
        );
  }
}
