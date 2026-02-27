package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.TurretConstants;
import frc.robot.util.ShooterOffsetMap;

public class SimFuelIKSubsystem {
    private static final double GRAVITY = 9.80665;

    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> speedsSupplier;
    private final ShooterOffsetMap offsetMap = new ShooterOffsetMap();

    public SimFuelIKSubsystem(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedsSupplier) {
        this.poseSupplier = poseSupplier;
        this.speedsSupplier = speedsSupplier;
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
        double alpha = Math.atan2(deltaHeight, distanceMeters);
        double thetaOptimal = 0.5 * (alpha + (Math.PI / 2.0));

        double minAngle = Math.toRadians(TurretConstants.hoodMinDegrees);
        double maxAngle = Math.toRadians(TurretConstants.hoodMaxDegrees);
        double thetaClamped = clamp(thetaOptimal, minAngle, maxAngle);

        double bestTheta = Double.NaN;
        double bestMotorRps = Double.POSITIVE_INFINITY;

        double[] candidates = {thetaClamped, minAngle, maxAngle};
        for (double theta : candidates) {
            double speedMps = solveSpeedFromEquation(distanceMeters, alpha, theta);
            if (!Double.isFinite(speedMps) || speedMps <= 0.0) {
                continue;
            }
            double wheelRps = speedMps / (2.0 * Math.PI * TurretConstants.shooterWheelRadius);
            double motorRps = wheelRps * TurretConstants.shooterRatio;
            if (motorRps <= TurretConstants.shooterMaxMotorRps && motorRps < bestMotorRps) {
                bestMotorRps = motorRps;
                bestTheta = theta;
            }
        }

        if (!Double.isFinite(bestTheta)) {
            return null;
        }

        ShooterOffsetMap.Offsets offsets = offsetMap.sample(distanceMeters);
        double hoodDeg = clamp(
            Math.toDegrees(bestTheta) + offsets.hoodOffsetDeg,
            TurretConstants.hoodMinDegrees,
            TurretConstants.hoodMaxDegrees
        );
        double motorRps = clamp(
            bestMotorRps + offsets.motorRpsOffset,
            0.0,
            TurretConstants.shooterMaxMotorRps
        );
        return new ShotSolution(hoodDeg, motorRps);
    }

    private static double solveSpeedFromEquation(double distanceMeters, double alpha, double theta) {
        double denominator = 2.0 * Math.cos(theta) * Math.sin(theta - alpha);
        if (denominator <= 1e-9) {
            return Double.NaN;
        }
        double vSquared = (GRAVITY * distanceMeters * Math.cos(alpha)) / denominator;
        if (vSquared <= 0.0) {
            return Double.NaN;
        }
        return Math.sqrt(vSquared);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
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
