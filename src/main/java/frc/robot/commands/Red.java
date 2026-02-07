// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Red extends Command {
  private LEDSubsystem ledSubsystem = new LEDSubsystem();
  private Supplier<Boolean> a;
  /** Creates a new Red. */
  // public Red(LEDSubsystem ledSubsystem) {
  //   // Use addRequirements() here to declare subsystem dependencies.
    
  // }

  public Red(LEDSubsystem ledSubsystem,Supplier<Boolean> a) {
    //TODO Auto-generated constructor stub
    this.ledSubsystem = ledSubsystem;
    this.a = a;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // ledSubsystem.rainbow()//solidColor(  new RGBWColor(255, 0, 0, 0)
        ledSubsystem.solidColor(new RGBWColor(255, 0, 0, 0));

;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ledSubsystem.empty();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !a.get();
    // return false;
  }
}
