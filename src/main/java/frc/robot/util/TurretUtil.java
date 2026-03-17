package frc.robot.util;

import java.math.BigInteger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.TurretConstants;

public final class TurretUtil {
    /**
    * Calculates the true mathematical (floored) modulo for doubles.
    * The result has the same sign as the divisor (y).
    */
    public static double floorMod(double x, double y) {
        // Formula: x - y * floor(x / y)
        return x - y * Math.floor(x / y);
    }

    public static double modInverse(double a, double m) {
        // 1. Convert doubles to BigIntegers
        BigInteger bigA = BigInteger.valueOf((long) a);
        BigInteger bigM = BigInteger.valueOf((long) m);
        
        // 2. Use the built-in modInverse (uses Extended Euclidean Algorithm)
        // This will throw an ArithmeticException if gcd(a, m) != 1
        BigInteger result = bigA.modInverse(bigM);
        
        return result.doubleValue();
    }

    // Normalizes to [-pi, pi]
    public static double normalizeRadians(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    public static boolean closerPoint(Pose2d pose, Translation2d poseLeft, Translation2d poseRight) {
        return Math.hypot(poseLeft.getX() - pose.getX(), poseLeft.getY() - pose.getY()) < 
            Math.hypot(poseRight.getX() - pose.getX(), poseRight.getY() - pose.getY());
    }

    public static double hoodDegreesToRotations(double hoodDegrees) {
        return ((hoodDegrees - TurretConstants.hoodZeroDegrees) / 360.0) * TurretConstants.hoodRatio;
    }

    public static double hoodRotationsToDegrees(double hoodRotations) {
        return TurretConstants.hoodZeroDegrees + ((hoodRotations / TurretConstants.hoodRatio) * 360.0);
    }

    public static double motorRpsToLaunchSpeedMps(double motorRps) {
        return ((Math.PI * 2) * TurretConstants.shooterWheelRadius)
            * motorRps
            * TurretConstants.shooterLaunchEfficiency;
    }

    public static double launchMpsToMotorRps(double launchSpeedMps) {
        double denominator = ((Math.PI * 2) * TurretConstants.shooterWheelRadius)
            * TurretConstants.shooterLaunchEfficiency;
        if (denominator <= 1e-9) {
            return 0.0;
        }
        return launchSpeedMps / denominator;
    }

    private static double timeOfFlight(double shooterRps, double hoodRadians, double distanceMeters) {
        double shooterMps = motorRpsToLaunchSpeedMps(shooterRps);
        double horizontalMps = shooterMps * Math.cos(hoodRadians);
        if (horizontalMps <= 1e-6) {
            return 0.0;
        }
        double tof = distanceMeters / horizontalMps;
        if (!Double.isFinite(tof) || tof < 0.0) {
            return 0.0;
        }
        return tof;
    }

    public static double tofFromMap(ShooterParams params, double distanceMeters) {
        double hoodRadians = Math.toRadians(90.0 - hoodRotationsToDegrees(params.hoodPose));
        return timeOfFlight(params.shooterSpeed, hoodRadians, distanceMeters);
    }

    public static double tofFromIK(double motorRps, double hoodDeg, double distanceMeters) {
        double hoodRadians = Math.toRadians(90.0 - hoodDeg);
        return timeOfFlight(motorRps, hoodRadians, distanceMeters);
    }
}
