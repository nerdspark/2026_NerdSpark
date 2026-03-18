package frc.robot.commands;

import static edu.wpi.first.units.Units.Value;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeJitterCommand extends Command {

    Intake intake;
    Supplier<Boolean> isActive;
    Supplier<Double> lowerShakePos;
    Supplier<Double> upperShakePos;
    Supplier<Double> shakeTarget;
    Timer timer;


    public IntakeJitterCommand(Intake intake, Supplier<Boolean> isActive, Supplier<Double> lowerShakePos, Supplier<Double> upperShakePos) {
        this.intake = intake;
        this.isActive = isActive;
        this.lowerShakePos = lowerShakePos;
        this.upperShakePos = upperShakePos;
        this.shakeTarget = () -> IntakeConstants.upperShakePos;
        this.timer = new Timer(); // redundant? idk the this keyword is for better readability

        addRequirements(intake);
    }

    public void changeShakePos(double value) {
        shakeTarget = () -> value;
    }

    public void jitter() {
        if (timer.get() < 0.15) { // activate if the timer is a whole number
            changeShakePos(upperShakePos.get().doubleValue());
        }
        else if(timer.get() > 0.15 && timer.get() < 0.3) { 
            changeShakePos(lowerShakePos.get().doubleValue());

        }
        else { 
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