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
        return Math.hypot(pose.getX() - poseLeft.getX(), pose.getY() - poseLeft.getY()) < 
            Math.hypot(pose.getX() - poseRight.getX(), pose.getY() - poseRight.getY());
    }

    public static double hoodDegreesToRotations(double hoodDegrees) {
        return (hoodDegrees / 360.0) * TurretConstants.hoodRatio;
    }
}