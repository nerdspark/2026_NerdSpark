// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LEDSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlinkBlue extends InstantCommand {
  private LEDSubsystem ledSubsystem = new LEDSubsystem();
  private Supplier<Boolean> b;
  /** Creates a new blue. */
  public BlinkBlue(LEDSubsystem ledSubsystem, Supplier<Boolean> b) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ledSubsystem = ledSubsystem;
    this.b = b;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ledSubsystem.setClimbing(b.get());
  }
}
