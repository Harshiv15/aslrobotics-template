package frc.robot.OI;

import static edu.wpi.first.wpilibj2.command.Commands.none;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface OperatorMap {
  Trigger shoot();

  default Command rumble() {
    return none();
  }

  Trigger prepShot();

  Trigger intake();
}
