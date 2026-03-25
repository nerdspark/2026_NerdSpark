package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TurretConstants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.function.Supplier;

/**
 * Drives the robot to the ideal fixed-angle shooting band: a pose at
 * {@code Turret/FixedAngle/BandDistanceMeters} from the hub, on the straight
 * line between the robot's current position and the hub, facing the hub.
 *
 * Use in combination with {@link frc.robot.subsystems.Turret#solveWithRequiredAngle}
 * as a backup shooting mode when the primary IK solution is unavailable.
 */
public class DriveToShootBand extends Command {

    private final DriveToPose inner;

    public DriveToShootBand(CommandSwerveDrivetrain drive, Supplier<Pose2d> robotPose) {
        inner = new DriveToPose(drive, () -> computeTarget(robotPose.get()));
    }

    private static Pose2d computeTarget(Pose2d robotPose) {
        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue;
        Translation2d hub = isBlue
            ? FieldConstants.Hub.topCenterPoint.toTranslation2d()
            : FieldConstants.Hub.oppTopCenterPoint.toTranslation2d();

        double idealDistance = SmartDashboard.getNumber(
            TurretConstants.fixedAngleBandDistanceKey,
            TurretConstants.fixedAngleBandDistanceDefault
        );
        Translation2d toRobot = robotPose.getTranslation().minus(hub);
        double dist = toRobot.getNorm();
        if (dist < 1e-3) {
            toRobot = new Translation2d(1.0, 0.0);
            dist = 1.0;
        }
        Translation2d targetTranslation = hub.plus(toRobot.times(idealDistance / dist));
        Translation2d toHub = hub.minus(targetTranslation);
        Rotation2d facingHub = new Rotation2d(toHub.getX(), toHub.getY());

        return new Pose2d(targetTranslation, facingHub);
    }

    @Override
    public void initialize() {
        inner.initialize();
    }

    @Override
    public void execute() {
        inner.execute();
    }

    @Override
    public void end(boolean interrupted) {
        inner.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return inner.isFinished();
    }

    public boolean atGoal() {
        return inner.atGoal();
    }
}
