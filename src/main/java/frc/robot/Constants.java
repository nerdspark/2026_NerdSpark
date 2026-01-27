package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public final class Constants {
    public static final class TurretConstants {
        public static final double spinRatio = 210/21.0;
        public static final double spinTeeth = 210;
        public static final double spinCancoder1Teeth = 15;
        public static final double spinCancoder2Teeth = 14; 

        public static final double shooterRatio = 2; // TODO
        public static final double shooterWheelRadius = 0.0508; // in meters TODO

        public static final double hoodStowPose = 0.0; // TODO

        // Hood pose from each drive to pose position
        public static final double[] hoodMap = {0, 0, 0, 0};
        // wheel speed from each drive to pose position
        public static final double[] wheelMap = {0, 0, 0, 0};
        
        public static InterpolatingTreeMap<Double, double[]> map =  new InterpolatingTreeMap<>(
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

        // Add numbers to hash map here, Distance, [Hood Pose, Wheel Speed (motor RPS)]
        static {
            map.put(0.0, new double[] {0, 0});
        }
    }

    public static final class TurretConfig {
        public static final String CANbus = "canivore1";

        public static final int spinMotorId = 25;
        public static final double spinKp = 3;
        public static final double spinKi = 0.0;
        public static final double spinKd = 0.08;
        public static final double spinKs = 0.0;
        public static final double spinKv = 0.0;
        public static final double spinKa = 0.0;
        public static final double spinStatorCurrentLimit = 40.0;
        public static final double spinVelocity = 40;
        public static final double spinAccel = 9000;

        public static final int spinCancoder1Id = 26;
        public static final double spinCancoder1Offset = -0.241455078125;

        public static final int spinCancoder2Id = 27;
        public static final double spinCancoder2Offset = -0.10009765625;
        
        public static final int hoodMotorId = 28;
        public static final double hoodKp = 0.0;
        public static final double hoodKi = 0.0;
        public static final double hoodKd = 0.0;
        public static final double hoodKs = 0.0;
        public static final double hoodKv = 0.0;
        public static final double hoodKa = 0.0;
        public static final double hoodStatorCurrentLimit = 40.0;

        public static final int shootMotorId = 29;
        public static final double shootKp = 0.0;
        public static final double shootKi = 0.0;
        public static final double shootKd = 0.0;
        public static final double shootKs = 0.0;
        public static final double shootKv = 0.0;
        public static final double shootKa = 0.0;
        public static final double shootStatorCurrentLimit = 60.0;
    }

    public static final class Field {
        public static final Translation2d blueHub = new Translation2d(4.615, 4.040); // May need to redo
        public static final Translation2d redHub = new Translation2d(11.915, 4.040); // May need to redo

        public static final double blueHubMaxX = 4.5;
        public static final double redHubMinX = 12.0;

        public static final Translation2d leftPass = new Translation2d(); // TODO
        public static final Translation2d rightPass = new Translation2d(); // TODO
    }

    public static final class TurretTelemetryConstants {
        public static final String angleRadKey = "Turret/AngleRad";
        public static final String angleDegKey = "Turret/AngleDeg";
        public static final String errorRadKey = "Turret/ErrorRad";
        public static final String errorDegKey = "Turret/ErrorDeg";
        public static final String setpointRadKey = "Turret/SetpointRad";
        public static final String setpointDegKey = "Turret/SetpointDeg";
        public static final String spinAngleDegKey = "Turret/SpinAngleRot";
        public static final String spinClosedLoopOutputKey = "Turret/SpinClosedLoopOutput";
        public static final String spinMotorVoltsKey = "Turret/SpinMotorVolts";

        public static final String hoodAngleDegKey = "Hood/AngleDeg";
        public static final String hoodSetpointRotKey = "Hood/SetpointRot";
        public static final String hoodClosedLoopOutputKey = "Hood/ClosedLoopOutput";
        public static final String hoodMotorVoltsKey = "Hood/MotorVolts";

        public static final String shooterSetpointRpsKey = "Shooter/SetpointRps";
        public static final String shooterMotorRpsKey = "Shooter/MotorRps";
        public static final String shooterWheelRpsKey = "Shooter/WheelRps";
        public static final String shooterMotorVoltsKey = "Shooter/MotorVolts";
    }
}