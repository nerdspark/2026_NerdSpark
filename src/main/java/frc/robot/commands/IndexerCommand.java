package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

public class IndexerCommand extends Command {

    Indexer indexer;
    Supplier<Boolean> isActive;

    public IndexerCommand(Indexer indexer, Supplier<Boolean> isActive) {
        this.indexer = indexer;
        this.isActive = isActive;

        addRequirements(indexer);
    }

     @Override
    public void initialize() {
       
    }

    @Override
    public void execute() {
        indexer.passThrough(isActive);
        indexer.moveDrumMotors(isActive);
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
