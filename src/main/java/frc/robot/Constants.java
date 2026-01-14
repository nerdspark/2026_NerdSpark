package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;

public class Constants {
    public final class turretConstants {
        public static final double spinOverrallRatio = 20;

        public static final double hoodStow = 0.0;
        
        public static InterpolatingTreeMap<Double, double[]> turretMap =  new InterpolatingTreeMap<>(
            // inverseInterpolator for Double keys
            (lower, upper, query) -> (query - lower) / (upper - lower),

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
