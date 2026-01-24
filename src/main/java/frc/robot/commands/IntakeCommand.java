package frc.robot.commands;
import com.ctre.phoenix6.Utils;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command{
    
    Intake intake;

    public IntakeCommand(Intake intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    public void initialize() {

    }

    public void execute() {
        intake.setDeployPower(0.5);
        intake.setRollerPower(0.8);
    }

    public void end(boolean interrupted) {

    }

    public boolean isFinished() {
        return false;
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    } 


}
