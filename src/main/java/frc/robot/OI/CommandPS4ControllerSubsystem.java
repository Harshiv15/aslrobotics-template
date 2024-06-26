// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.OI;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import java.util.function.DoubleSupplier;

/** A CommandPS4Controller that implements Subsystem to allow the rumble to be mutexed. */
public class CommandPS4ControllerSubsystem extends CommandPS4Controller implements Subsystem {

  public CommandPS4ControllerSubsystem(int port) {
    super(port);
  }

  /** Rumble the controller at the specified power. */
  public Command rumbleCmd(DoubleSupplier left, DoubleSupplier right) {
    return this.run(
        () -> {
          super.getHID().setRumble(RumbleType.kLeftRumble, left.getAsDouble());
          super.getHID().setRumble(RumbleType.kRightRumble, right.getAsDouble());
        });
  }
}
