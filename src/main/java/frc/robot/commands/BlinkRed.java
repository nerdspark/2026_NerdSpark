// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BlinkRed extends Command {
  private LEDSubsystem ledSubsystem = new LEDSubsystem();
  private Supplier<Boolean> b;
  /** Creates a new blue. */
  public BlinkRed(LEDSubsystem ledSubsystem, Supplier<Boolean> b) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ledSubsystem = ledSubsystem;
    this.b = b;
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ledSubsystem.blinkColor(new RGBWColor(0, 0, 255, 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // ledSubsystem.solidColor(new RGBWColor(0, 0, 0, 0));
    ledSubsystem.empty();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return false;  
    return !b.get();  
  }
}
