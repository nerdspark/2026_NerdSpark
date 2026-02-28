package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

public class IndexerCommand extends Command {
    private final Indexer indexer;
    private final Supplier<Boolean> isActive;
    private final Supplier<Double> rollerSpeed;

    public IndexerCommand(Indexer indexer, Supplier<Boolean> isActive, Supplier<Double> rollerSpeed) {
        this.indexer = indexer;
        this.isActive = isActive;
        this.rollerSpeed = rollerSpeed;
        addRequirements(indexer);
    }

    public IndexerCommand(Indexer indexer, Supplier<Double> rollerSpeed) {
        this(indexer, () -> true, rollerSpeed);
    }

    @Override
    public void execute() {
        if (isActive.get()) {
            indexer.passThrough(rollerSpeed);
        } else {
            indexer.stopPassThrough();
        }
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
