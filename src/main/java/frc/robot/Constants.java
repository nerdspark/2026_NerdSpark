// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double gyroP = 2;
    public static final double gyroI = 0.0;
    public static final double gyroD = 0.00;

    public static final String pigeonCanBus = "canivore1";

    public static class Vision {
        public static boolean DOGLOG_ENABLED = false;

        public static final boolean USE_VISION = true;

        public static final String kCameraNameFrontRight = "FrontRightCamera";
        public static final Transform3d kRobotToCamFrontRight =
            new Transform3d(new Translation3d(Units.inchesToMeters(11.5), -Units.inchesToMeters((9)), Units.inchesToMeters(12.5)), 
                new Rotation3d(Math.toRadians(0), Math.toRadians(-10), Math.toRadians(0))); //TODO: determine XYZ

        public static final String kCameraNameFrontLeft = "FrontLeftCamera";
        public static final Transform3d kRobotToCamFrontLeft =
            new Transform3d(new Translation3d(Units.inchesToMeters(11.5), Units.inchesToMeters((9)), Units.inchesToMeters(12.5)), 
                new Rotation3d(Math.toRadians(0), Math.toRadians(-25), Math.toRadians(55))); //TODO: determine XYZ

        public static final String kCameraNameBackRight = "BackRightCamera";
        public static final Transform3d kRobotToCamBackRight =
            new Transform3d(new Translation3d(-Units.inchesToMeters(11.5), -Units.inchesToMeters((9)), Units.inchesToMeters(12.5)), 
                new Rotation3d(Math.toRadians(0), Math.toRadians(-10), Math.toRadians(180))); //TODO: determine XYZ
    
        public static final String kCameraNameBackLeft = "BackLeftCamera";
        public static final Transform3d kRobotToCamBackLeft =
            new Transform3d(new Translation3d(-Units.inchesToMeters(11.5), Units.inchesToMeters((9)), Units.inchesToMeters(12.5)), 
                new Rotation3d(Math.toRadians(0), Math.toRadians(-25), Math.toRadians(125))); //TODO: determine XYZ
        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        // Do not change these. Actual values will be calculated by the vision system.
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);

        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
            
        // Change these for fine tune vision system calculations of standard deviations.
        public static final double kXYStdDev = 0.4; 
        public static final double kThetaStdDev = 1; 

        public static final double TRANSLATION_TOLERANCE_X = 0.013; // Changed from 0.05 3/8/25
        public static final double TRANSLATION_TOLERANCE_Y = 0.013; // Changed from 0.05 3/8/25
        public static final double ROTATION_TOLERANCE = Math.toRadians(1.3); // /deg

        // Below same as pathplanner constants
        public static final double MAX_VELOCITY = 3; 
        public static final double MAX_ACCELERATION = 5; 
        public static final double MAX_VELOCITY_ROTATION = 540; 
        public static final double MAX_ACCELARATION_ROTATION = 720;
            
        public static final double VELOCITY_TOLERANCE_X = 4;
        public static final double VELOCITY_TOLERANCE_Y = 4;
        public static final double VELOCITY_TOLERANCE_OMEGA = 5;

        public static final double kPXController = 15; //2.5
        public static final double kIXController = 0.0 ; //0.01d
        public static final double kDXController = 0.1d;
        
        public static final double kPThetaController = 7; //2
        public static final double kIThetaController = 0.0;
        public static final double kDThetaController = 0.0; //0.0041

        public static final double kPoseAmbiguityThreshold = 0.2;
        public static final double kSingleTagDistanceThreshold = 2.0;

            
    }

    public static final class TurretConstants {
        public static final double spinRatio = 210/21.0;
        public static final double spinTeeth = 210;
        public static final double spinCancoder1Teeth = 15;
        public static final double spinCancoder2Teeth = 14;
        
        public static final double hoodRatio = 50;

        public static final double shooterWheelRadius = 0.0508; // in meters TODO

        public static final double hoodStowPose = 0.0;

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
        
        public static final int hoodMotor1Id = 28;
        public static final int hoodMotor2Id = 29;
        public static final double hoodKp1 = 0.0;
        public static final double hoodKp2 = 0.0;
        public static final double hoodKi1 = 0.0;
        public static final double hoodKi2 = 0.0;
        public static final double hoodKd1 = 0.0;
        public static final double hoodKd2 = 0.0;
        public static final double hoodKs1 = 0.0;
        public static final double hoodKs2 = 0.0;
        public static final double hoodKv1 = 0.0;
        public static final double hoodKv2 = 0.0;
        public static final double hoodKa1 = 0.0;
        public static final double hoodKa2 = 0.0;
        public static final double hoodStatorCurrentLimit = 40.0;

        public static final int shootMotor1Id = 30;
        public static final int shootMotor2Id = 31;
        public static final double shootKp1 = 0.0;
        public static final double shootKp2 = 0.0;
        public static final double shootKi1 = 0.0;
        public static final double shootKi2 = 0.0;
        public static final double shootKd1 = 0.0;
        public static final double shootKd2 = 0.0;
        public static final double shootKs1 = 0.0;
        public static final double shootKs2 = 0.0;
        public static final double shootKv1 = 0.0;
        public static final double shootKv2 = 0.0;
        public static final double shootKa1 = 0.0;
        public static final double shootKa2 = 0.0;
        public static final double shootStatorCurrentLimit = 60.0;
    }

    public static final class Field {
        public static final Translation2d blueHub = new Translation2d(4.615, 4.040); // May need to redo
        public static final Translation2d redHub = new Translation2d(11.915, 4.040); // May need to redo

        public static final double blueHubMaxX = 4.5; // Double check
        public static final double redHubMinX = 12.0; // Double check

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