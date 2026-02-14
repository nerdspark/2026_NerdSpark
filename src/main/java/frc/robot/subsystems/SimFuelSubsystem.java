package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.util.FuelSim;

public class SimFuelSubsystem extends SubsystemBase {
    private static final double GRAVITY = 9.80665;
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> speedsSupplier;

    public SimFuelSubsystem(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedsSupplier) {
        this.poseSupplier = poseSupplier;
        this.speedsSupplier = speedsSupplier;
        if (RobotBase.isSimulation()) {
            configureFuelSim();
        }
    }

    private void configureFuelSim() {
        FuelSim instance = FuelSim.getInstance();
        instance.spawnStartingFuel();
        instance.registerRobot(
            Units.inchesToMeters(25),
            Units.inchesToMeters(29),
            Units.inchesToMeters(25),
            poseSupplier,
            speedsSupplier
        );
        instance.registerIntake(
            -Units.inchesToMeters(26),
            Units.inchesToMeters(26),
            -Units.inchesToMeters(26),
            Units.inchesToMeters(26),
            () -> true
        );

        instance.start();
        SmartDashboard.putData(
            Commands.runOnce(() -> {
                FuelSim.getInstance().clearFuel();
                FuelSim.getInstance().spawnStartingFuel();
            })
            .withName("Reset Fuel")
            .ignoringDisable(true)
        );
    }

    public Translation3d getRobotLaunchPosition() {
        Pose2d pose = poseSupplier.get();
        Translation2d turretTranslation = pose.getTranslation()
            .plus(TurretConstants.robotToTurret.rotateBy(pose.getRotation()));
        return new Translation3d(
            turretTranslation.getX(),
            turretTranslation.getY(),
            TurretConstants.shooterMuzzleHeightMeters
        );
    }

    public Translation3d launchVel(Translation3d fieldRelativeVelocity) {
        ChassisSpeeds fieldSpeeds = speedsSupplier.get();
        return new Translation3d(
            fieldRelativeVelocity.getX() + fieldSpeeds.vxMetersPerSecond,
            fieldRelativeVelocity.getY() + fieldSpeeds.vyMetersPerSecond,
            fieldRelativeVelocity.getZ()
        );
    }

    public Translation3d computeLaunchVelocityToTarget(Translation2d target) {
        Pose2d pose = poseSupplier.get();
        Translation2d turretTranslation = pose.getTranslation()
            .plus(TurretConstants.robotToTurret.rotateBy(pose.getRotation()));
        double dx = target.getX() - turretTranslation.getX();
        double dy = target.getY() - turretTranslation.getY();
        double distance = Math.hypot(dx, dy);
        if (distance <= 1e-6) {
            return new Translation3d();
        }

        ShotSolution solution = solveShotForDistance(distance);
        if (solution == null) {
            return new Translation3d();
        }

        double hoodRad = Math.toRadians(solution.hoodDegrees);
        double wheelRps = solution.motorRps / TurretConstants.shooterRatio;
        double muzzleSpeed = wheelRps * 2.0 * Math.PI * TurretConstants.shooterWheelRadius;
        double horizontalSpeed = muzzleSpeed * Math.cos(hoodRad);
        double verticalSpeed = muzzleSpeed * Math.sin(hoodRad);

        double unitX = dx / distance;
        double unitY = dy / distance;

        Translation3d fieldVelocity = new Translation3d(
            unitX * horizontalSpeed,
            unitY * horizontalSpeed,
            verticalSpeed
        );
        ChassisSpeeds fieldSpeeds = speedsSupplier.get();
        return new Translation3d(
            fieldVelocity.getX() - fieldSpeeds.vxMetersPerSecond,
            fieldVelocity.getY() - fieldSpeeds.vyMetersPerSecond,
            fieldVelocity.getZ()
        );
    }

    private ShotSolution solveShotForDistance(double distanceMeters) {
        if (distanceMeters <= 0.0) {
            return null;
        }
        double deltaHeight = TurretConstants.targetHeightMeters - TurretConstants.shooterMuzzleHeightMeters;
        double bestAngleDeg = Double.NaN;
        double bestMotorRps = Double.POSITIVE_INFINITY;
        double minAngle = TurretConstants.hoodMinDegrees;
        double maxAngle = TurretConstants.hoodMaxDegrees;

        for (double angleDeg = minAngle; angleDeg <= maxAngle; angleDeg += TurretConstants.shotAngleStepDeg) {
            double angleRad = Math.toRadians(angleDeg);
            double speedMps = solveBallisticSpeed(distanceMeters, angleRad, deltaHeight);
            if (!Double.isFinite(speedMps)) {
                continue;
            }
            double wheelRps = speedMps / (2.0 * Math.PI * TurretConstants.shooterWheelRadius);
            double motorRps = wheelRps * TurretConstants.shooterRatio;
            if (motorRps <= TurretConstants.shooterMaxMotorRps && motorRps < bestMotorRps) {
                bestMotorRps = motorRps;
                bestAngleDeg = angleDeg;
            }
        }

        if (!Double.isFinite(bestAngleDeg)) {
            return null;
        }
        return new ShotSolution(bestAngleDeg, bestMotorRps);
    }

    private static double solveBallisticSpeed(double distanceMeters, double angleRad, double deltaHeightMeters) {
        double cos = Math.cos(angleRad);
        if (Math.abs(cos) < 1e-6) {
            return Double.NaN;
        }
        double tan = Math.tan(angleRad);
        double denom = 2.0 * cos * cos * (distanceMeters * tan - deltaHeightMeters);
        if (denom <= 0.0) {
            return Double.NaN;
        }
        double numerator = GRAVITY * distanceMeters * distanceMeters;
        return Math.sqrt(numerator / denom);
    }

    private static final class ShotSolution {
        private final double hoodDegrees;
        private final double motorRps;

        private ShotSolution(double hoodDegrees, double motorRps) {
            this.hoodDegrees = hoodDegrees;
            this.motorRps = motorRps;
        }
    }
}
