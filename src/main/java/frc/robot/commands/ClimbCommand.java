// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commandSequences.ClimbSequences;
import frc.robot.subsystems.Climb.ShortClimb;
import frc.robot.subsystems.Climb.TallClimb;


//Start of CLimb Sequence
//Taller Arm goes to 30 inches and hooks on first rung and Small Arm goes to highest position
//Tall Arm pulls down until small arm then small arm hooks on first rung and Tall Arm lets go
//Tall Arm goes to second rung position and hooks on second rung
//Kicker Arm goes out and pushes robot back and allows clearance
//Tall Arm pulls down until small arm then small arm hooks on second rung and Tall Arm lets go
//Tall Arm goes to third rung position and hooks on third rung
//Kicker Arm goes out and pushes robot back and allow clearance
//Tall Arm pulls down until small arm then small arm hooks on third rung and Tall Arm lets go.

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbCommand extends Command {

  TallClimb tallClimb;
  ShortClimb shortClimb;
  Supplier<Double> positionLeft;
  Supplier<Double> positionRight;
  private Command activeSequence;

  /** Creates a new ClimbCommand. */
  public ClimbCommand(TallClimb tallClimb, ShortClimb shortClimb, Supplier<Double> positionLeft, Supplier<Double> positionRight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.tallClimb = tallClimb;
    this.shortClimb = shortClimb;
    this.positionLeft = positionLeft;
    this.positionRight = positionRight;
    addRequirements(tallClimb); // ONLY USE WHEN NOT DOING BUTTON BINDS
    addRequirements(shortClimb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // activeSequence = ClimbSequences.climbToL1(climb); // ONLY USE WHEN NOT DOING BUTTON BINDS
    // activeSequence.schedule(); // ONLY USE WHEN NOT DOING BUTTON BINDS
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // climb.setClimbLeft(positionLeft);
    // climb.setClimbRight(positionRight);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
