package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Indexer;
import edu.wpi.first.wpilibj.Timer;

public class IndexerJitterCommand extends InstantCommand {

    Indexer indexer;
    Supplier<Boolean> isActive;
    Supplier<Double> rollerSpeed;
    Timer timer;


    public IndexerJitterCommand(Indexer indexer, Supplier<Boolean> isActive, Supplier<Double> rollerSpeed) {
        this.indexer = indexer;
        this.isActive = isActive;
        this.rollerSpeed = rollerSpeed;

        this.timer = new Timer(); // redundant? idk the this keyword is for better readability

        addRequirements(indexer);
    }

    public void changeRollerSpeed(double value) {
        rollerSpeed = () -> value;
    }

    public void jitter() {
        if (timer.get() > 0.15) { // activate if the timer is a whole number
            changeRollerSpeed(-rollerSpeed.get()); // set roller speed to negative itself
            timer.reset();
        }
    }

    public Supplier<Double> getRollerSpeed() {
        return rollerSpeed;
    }

     @Override
    public void initialize() {
        timer.reset();
        timer.start(); 
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
