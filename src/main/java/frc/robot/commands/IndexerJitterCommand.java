package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Indexer;
import edu.wpi.first.wpilibj.Timer;

public class IndexerJitterCommand extends InstantCommand {

    Indexer indexer;
    Supplier<Boolean> isActive;
    Supplier<Double> rollerSpeed;


    public IndexerJitterCommand(Indexer indexer, Supplier<Boolean> isActive, Supplier<Double> rollerSpeed) {
        this.indexer = indexer;
        this.isActive = isActive;
        this.rollerSpeed = rollerSpeed;

        addRequirements(indexer);
    }

    public void changeRollerSpeed(double value){
        rollerSpeed = () -> value;
    }

    public void jitter() {
        if (Timer.getFPGATimestamp() % 1 == 0) {
            changeRollerSpeed(-rollerSpeed.get());
        }
    }

    public Supplier<Double> getRollerSpeed(){
        return rollerSpeed;
    }

     @Override
    public void initialize() {
       
    }

    @Override
    public void execute() {
        jitter();
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
