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
        double muzzleSpeed = solution.motorRps * 2.0 * Math.PI * TurretConstants.shooterWheelRadius;
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
        double minAngleDeg = TurretConstants.hoodMinDegrees;
        double maxAngleDeg = TurretConstants.hoodMaxDegrees;
        double targetEntryDeg = TurretConstants.ikEntryAngleTargetDeg;
        double toleranceDeg = TurretConstants.ikEntryAngleToleranceDeg;

        double bestBandAngleDeg = Double.NaN;
        double bestBandMotorRps = Double.POSITIVE_INFINITY;
        double bestBandEntryErrorDeg = Double.POSITIVE_INFINITY;
        double bestFallbackAngleDeg = Double.NaN;
        double bestFallbackMotorRps = Double.POSITIVE_INFINITY;

        for (double angleDeg = minAngleDeg; angleDeg <= maxAngleDeg; angleDeg += TurretConstants.shotAngleStepDeg) {
            double angleRad = Math.toRadians(angleDeg);
            double speedMps = solveSpeedFromEquation(distanceMeters, angleRad, deltaHeight);
            if (!Double.isFinite(speedMps) || speedMps <= 0.0) {
                continue;
            }

            double motorRps = speedMps / (2.0 * Math.PI * TurretConstants.shooterWheelRadius);
            if (motorRps > TurretConstants.shooterMaxMotorRps) {
                continue;
            }

            if (motorRps < bestFallbackMotorRps) {
                bestFallbackMotorRps = motorRps;
                bestFallbackAngleDeg = angleDeg;
            }

            double entryDeg = computeEntryAngleDeg(distanceMeters, speedMps, angleRad);
            if (!Double.isFinite(entryDeg)) {
                continue;
            }

            double entryErrorDeg = Math.abs(entryDeg - targetEntryDeg);
            if (entryErrorDeg <= toleranceDeg) {
                boolean betterRps = motorRps < bestBandMotorRps;
                boolean tieBreak = Math.abs(motorRps - bestBandMotorRps) < 1e-9
                    && entryErrorDeg < bestBandEntryErrorDeg;
                if (betterRps || tieBreak) {
                    bestBandMotorRps = motorRps;
                    bestBandAngleDeg = angleDeg;
                    bestBandEntryErrorDeg = entryErrorDeg;
                }
            }
        }

        double selectedAngleDeg = Double.isFinite(bestBandAngleDeg) ? bestBandAngleDeg : bestFallbackAngleDeg;
        double selectedMotorRps = Double.isFinite(bestBandAngleDeg) ? bestBandMotorRps : bestFallbackMotorRps;
        if (!Double.isFinite(selectedAngleDeg) || !Double.isFinite(selectedMotorRps)) {
            return null;
        }

        ShooterOffsetMap.Offsets offsets = offsetMap.sample(distanceMeters);
        double hoodDeg = clamp(
            selectedAngleDeg + offsets.hoodOffsetDeg,
            TurretConstants.hoodMinDegrees,
            TurretConstants.hoodMaxDegrees
        );
        double motorRps = clamp(
            selectedMotorRps + offsets.motorRpsOffset,
            0.0,
            TurretConstants.shooterMaxMotorRps
        );
        return new ShotSolution(hoodDeg, motorRps);
    }

    private static double solveSpeedFromEquation(double distanceMeters, double thetaRad, double deltaHeightMeters) {
        double cos = Math.cos(thetaRad);
        if (Math.abs(cos) < 1e-6) {
            return Double.NaN;
        }
        double tan = Math.tan(thetaRad);
        double denominator = 2.0 * cos * cos * (distanceMeters * tan - deltaHeightMeters);
        if (denominator <= 0.0) {
            return Double.NaN;
        }
        double vSquared = (GRAVITY * distanceMeters * distanceMeters) / denominator;
        if (vSquared <= 0.0) {
            return Double.NaN;
        }
        return Math.sqrt(vSquared);
    }

    private static double computeEntryAngleDeg(double distanceMeters, double launchSpeedMps, double launchAngleRad) {
        double horizontalSpeed = launchSpeedMps * Math.cos(launchAngleRad);
        if (horizontalSpeed <= 1e-6) {
            return Double.NaN;
        }
        double time = distanceMeters / horizontalSpeed;
        if (!Double.isFinite(time) || time <= 0.0) {
            return Double.NaN;
        }
        double verticalVelocityAtTarget = launchSpeedMps * Math.sin(launchAngleRad) - GRAVITY * time;
        return Math.toDegrees(Math.atan2(-verticalVelocityAtTarget, horizontalSpeed));
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
