package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Indexer;

public class IndexerCommand extends InstantCommand {

    Indexer indexer;
    Supplier<Double> rollerSpeed;
    Supplier<Boolean> index;

    public IndexerCommand(Indexer indexer, Supplier<Double> rollerSpeed, Supplier<Boolean> index) {
        this.indexer = indexer;
        this.rollerSpeed = rollerSpeed;
        this.index = index;

        addRequirements(indexer);
    }

    public void changeRollerSpeed(double value){
        rollerSpeed = () -> value;
    }

    public Supplier<Double> getRollerSpeed(){
        return rollerSpeed;
    }

     @Override
    public void initialize() {
       
    }

    @Override
    public void execute() {
        // indexer.setIsActive(isActive.get().booleanValue());
        if (index.get()) {
            indexer.spinDex(rollerSpeed);
        } else {
            indexer.spinDex(() -> 0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    }

}
