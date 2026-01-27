package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command{
    
    Intake intake;

    public IntakeCommand(Intake intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    public void initialize() {

    }

    public void execute() {
        intake.setDeployPower(1.0);
        intake.setRollerPower(1.0);
    }

    public void end(boolean interrupted) {

    }

    public boolean isFinished() {
        return false;
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    } 


}
