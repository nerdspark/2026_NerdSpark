package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.VelocityVoltage;

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
        // Step 1: desired surface speed
        double surfaceSpeed = chassisSpeed * 1.5;

        // Step 2: convert to roller RPS
        double rollerRPS = surfaceSpeed / (Math.PI * Units.inchesToMeters(2));

        // Step 3: convert to motor RPS
        double motorRPS = rollerRPS;// * GEAR_RATIO;

        // Step 4: set velocity
        intake.setRollerPower(motorRPS);
    }

    public void end(boolean interrupted) {
        intake.stopIntake();    
    }

    public boolean isFinished() {
        return false;
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    } 


}
