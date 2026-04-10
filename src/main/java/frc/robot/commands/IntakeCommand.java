package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command{
    
    Intake intake;
    Supplier<ChassisSpeeds> speeds;

    public IntakeCommand(Intake intake, Supplier<ChassisSpeeds> speeds) {
        this.intake = intake;
        this.speeds = speeds;

        addRequirements(intake);
    }

    public void initialize() {
    }

    public void execute() {
        double chassisSpeed = Math.hypot(speeds.get().vxMetersPerSecond, speeds.get().vyMetersPerSecond);
        chassisSpeed = Math.abs(chassisSpeed);
        // Step 1: desired surface speed
        double surfaceSpeed = chassisSpeed * 2.0;

        // Step 2: convert to roller RPS
        double rollerRPS = surfaceSpeed / (Math.PI * Units.inchesToMeters(3));
        rollerRPS *= 2.5;

        if (rollerRPS < 35) {
            rollerRPS = 25;
        }

        // Step 4: set velocity
        intake.setRollerSpeed(rollerRPS);
    }

    public void end(boolean interrupted) {
        intake.setRollerSpeed(0);    
    }

    public boolean isFinished() {
        return false;
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    } 


}
