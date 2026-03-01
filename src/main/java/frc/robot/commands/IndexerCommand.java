package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Indexer;

public class IndexerCommand extends InstantCommand {

    Indexer indexer;
    Supplier<Boolean> isActive;
    Supplier<Double> rollerSpeed;

    public IndexerCommand(Indexer indexer, Supplier<Boolean> isActive, Supplier<Double> rollerSpeed) {
        this.indexer = indexer;
        this.isActive = isActive;
        this.rollerSpeed = rollerSpeed;

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
        indexer.spinDex(rollerSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        
    }

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
