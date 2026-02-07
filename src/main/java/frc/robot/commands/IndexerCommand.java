package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Indexer;

public class IndexerCommand extends Command {

    Indexer indexer;
    Supplier<Double> rollerSpeed;

    public IndexerCommand(Indexer indexer, Supplier<Double> rollerSpeed) {
        this.indexer = indexer;
        this.rollerSpeed = rollerSpeed;

        addRequirements(indexer);
    }

     @Override
    public void initialize() {}

    @Override
    public void execute() {
        indexer.passThrough(rollerSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stopPassThrough();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    }

}
