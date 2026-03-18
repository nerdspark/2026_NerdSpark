package frc.robot.subsystems;

import static frc.robot.util.TurretUtil.motorRpsToLaunchSpeedMps;
import static frc.robot.util.TurretUtil.launchMpsToMotorRps;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoAimConstants;
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
            getConfiguredMuzzleHeightMeters()
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
        double muzzleSpeed = motorRpsToLaunchSpeedMps(solution.motorRps);
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

        boolean useEntryAngleIK = SmartDashboard.getBoolean(
            AutoAimConstants.useEntryAngleIKKey,
            AutoAimConstants.defaultUseEntryAngleIK
        );
        double deltaHeight = getConfiguredTargetHeightMeters() - getConfiguredMuzzleHeightMeters();

        DirectShotSelection selection = useEntryAngleIK
            ? solveEntryAngleIKDirect(distanceMeters, deltaHeight)
            : solveMinimumSpeedIKDirect(distanceMeters, deltaHeight);
        if (selection == null) {
            return null;
        }

        ShooterOffsetMap.Offsets offsets = offsetMap.sample(distanceMeters);
        double hoodDeg = clamp(
            selection.hoodDegrees + offsets.hoodOffsetDeg,
            TurretConstants.hoodMinDegrees,
            TurretConstants.hoodMaxDegrees
        );
        double motorRps = clamp(
            selection.motorRps + offsets.motorRpsOffset,
            0.0,
            useEntryAngleIK ? TurretConstants.shooterMaxMotorRps : Double.POSITIVE_INFINITY
        );
        return new ShotSolution(hoodDeg, motorRps);
    }

    private DirectShotSelection solveEntryAngleIKDirect(
        double distanceMeters,
        double deltaHeightMeters
    ) {
        double targetEntryRad = Math.toRadians(TurretConstants.ikEntryAngleTargetDeg);
        double desiredThetaRad = Math.atan(Math.tan(targetEntryRad) + (2.0 * deltaHeightMeters / distanceMeters));
        double desiredThetaDeg = Math.toDegrees(desiredThetaRad);
        if (!Double.isFinite(desiredThetaDeg)) {
            return solveMinimumSpeedIKDirect(distanceMeters, deltaHeightMeters);
        }
        if (desiredThetaDeg < TurretConstants.hoodMinDegrees || desiredThetaDeg > TurretConstants.hoodMaxDegrees) {
            return solveMinimumSpeedIKDirect(distanceMeters, deltaHeightMeters);
        }
        double speedMps = solveSpeedFromEquation(distanceMeters, Math.toRadians(desiredThetaDeg), deltaHeightMeters);
        double motorRps = launchMpsToMotorRps(speedMps);
        if (Double.isFinite(motorRps) && motorRps > 0.0 && motorRps <= TurretConstants.shooterMaxMotorRps) {
            return new DirectShotSelection(desiredThetaDeg, motorRps, true);
        }

        return solveMinimumSpeedIKDirect(distanceMeters, deltaHeightMeters);
    }

    private DirectShotSelection solveMinimumSpeedIKDirect(
        double distanceMeters,
        double deltaHeightMeters
    ) {
        double alphaRad = Math.atan2(deltaHeightMeters, distanceMeters);
        double thetaDeg = Math.toDegrees(0.5 * (alphaRad + (Math.PI / 2.0)));
        if (thetaDeg < TurretConstants.hoodMinDegrees || thetaDeg > TurretConstants.hoodMaxDegrees) {
            return null;
        }
        double speedMps = solveSpeedFromEquation(distanceMeters, Math.toRadians(thetaDeg), deltaHeightMeters);
        double motorRps = launchMpsToMotorRps(speedMps);
        if (!Double.isFinite(motorRps) || motorRps <= 0.0 || motorRps > TurretConstants.shooterMaxMotorRps) {
            return null;
        }
        return new DirectShotSelection(thetaDeg, motorRps, false);
    }

    private double getConfiguredMuzzleHeightMeters() {
        return SmartDashboard.getNumber(
            AutoAimConstants.modelMuzzleHeightMetersKey,
            TurretConstants.shooterMuzzleHeightMeters
        );
    }

    private double getConfiguredTargetHeightMeters() {
        return SmartDashboard.getNumber(
            AutoAimConstants.modelTargetHeightMetersKey,
            TurretConstants.targetHeightMeters
        );
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

    private static final class DirectShotSelection {
        private final double hoodDegrees;
        private final double motorRps;
        @SuppressWarnings("unused")
        private final boolean usedPrimaryObjective;

        private DirectShotSelection(double hoodDegrees, double motorRps, boolean usedPrimaryObjective) {
            this.hoodDegrees = hoodDegrees;
            this.motorRps = motorRps;
            this.usedPrimaryObjective = usedPrimaryObjective;
        }
    }
}
