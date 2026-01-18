package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class Constants {
    public final class turretConstants {
        public static final double spinRatio = 20;
        public static final double spinCancoder1Ratio = 10; // Turret gear teeth / encoder A gear teeth
        public static final double spinCancoder2Ratio = 11; // Turret gear teeth / encoder B gear teeth
        public static final double shooterRatio = 2;
        public static final double shooterWheelRadius = 0.0508; // in meters

        public static final double hoodStow = 0.0;
        public static final double turretOffset = 0;
        
        public static InterpolatingTreeMap<Double, double[]> turretMap =  new InterpolatingTreeMap<>(
            // inverseInterpolator for Double keys
            InverseInterpolator.forDouble(),

            // interpolator for Double[] values
            (lower, upper, t) -> {
                double[] result = new double[lower.length];
                for (int i = 0; i < lower.length; i++) {
                    result[i] = lower[i] + t * (upper[i] - lower[i]);
                }
                return result;
            }
        );

        // Add numbers to hash map here, Distance, [Hood Pose, Wheel Speed]
        static {
            // turretMap.put(, new double[] {});
        }
    }

    public final class field {
        public static final Translation2d blueHub = new Translation2d(4.615, 4.040); // May need to redo
        public static final Translation2d redHub = new Translation2d(11.915, 4.040); // May need to redo
    }
}