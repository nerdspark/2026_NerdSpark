package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;

public class IntakeJitterCommand extends InstantCommand {

    Intake intake;
    Supplier<Double> shakeTarget;
    Timer timer;
    boolean fast = false;


    public IntakeJitterCommand(Intake intake) {
        this.intake = intake;

        this.timer = new Timer(); // redundant? idk the this keyword is for better readability

        addRequirements(intake);
    }

    public void changeShakePos(double value) {
        shakeTarget = () -> value;
    }

    public void jitter() {
        if (timer.get() < 0.5) { // activate if the timer is a whole number
            changeShakePos(IntakeConstants.upperShakePos);
        } else if(timer.get() > 0.5 && timer.get() < 1) { 
            changeShakePos(IntakeConstants.lowerShakePos);
        } else { 
            timer.reset();
        }
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start(); 
    }

    @Override
    public void execute() {
        jitter();
        if (!fast) {
            intake.useFastConfig();
            fast = true;
        }
        intake.setDeployPosition(shakeTarget);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setDeployPosition(() -> IntakeConstants.deployPos);
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