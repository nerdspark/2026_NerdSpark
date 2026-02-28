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
public class UpdateLEDStatus extends InstantCommand {
  private String status;
  private String pastStatus;
    Supplier<String> statusSupplier;
    LEDSubsystem led;

  public UpdateLEDStatus(LEDSubsystem led, Supplier<String> statusSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.led = led;
    this.statusSupplier = statusSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    status = statusSupplier.get();
    pastStatus = led.getStatus();
    if (pastStatus != status) {
    led.setStatus(status);}
  }
}
