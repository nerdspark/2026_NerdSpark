package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.field;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret;

public class AimChassisCommand extends Command {
    private Turret turret;
    private CommandSwerveDrivetrain drivetrain;
    private double targetTheta;
    private final PhoenixPIDController thetaController = new PhoenixPIDController(0, 0, 0);
    private final ApplyRobotSpeeds driveRequest = new ApplyRobotSpeeds();
    
    public AimChassisCommand(Turret turret, CommandSwerveDrivetrain drivetrain) {
        this.turret = turret;
        this.drivetrain = drivetrain;
        thetaController.enableContinuousInput(-180, 180);

        addRequirements(turret, drivetrain);
    }

    @Override
    public void initialize() {
        thetaController.reset();
    }

    @Override
    public void execute() {
        Pose2d currPose = drivetrain.getState().Pose;
        
        double xError = Math.abs(field.blueHub.getX() - currPose.getX());
        double yError = Math.abs(field.blueHub.getY() - currPose.getY());
        double hubDegrees = normalize180(Math.toDegrees(Math.atan(yError/xError)));
        
        targetTheta = turret.aimChassis(hubDegrees, normalize180(currPose.getRotation().getDegrees()));

        double thetaVelocity = thetaController.calculate(currPose.getRotation().getDegrees(), targetTheta, Utils.getCurrentTimeSeconds());
        
        drivetrain.setControl(driveRequest.withSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
            0, 0, Math.toRadians(thetaVelocity), drivetrain.getState().Pose.getRotation()))
            .withDriveRequestType(DriveRequestType.Velocity)
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        double thetaError = targetTheta - drivetrain.getState().Pose.getRotation().getDegrees();
        return 1 < thetaError;
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    }

    private static double normalize180(double degrees) {
        degrees %= 360.0;        // wrap within -360..360
        if (degrees > 180.0) {
            degrees -= 360.0;    // move into -180..180
        } else if (degrees < -180.0) {
            degrees += 360.0;    // move into -180..180
        }
        return degrees;
    }
}
