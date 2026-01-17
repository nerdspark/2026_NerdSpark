// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.Map;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import edu.wpi.first.math.util.Units;
import java.lang.annotation.Documented;
import java.lang.reflect.Array;
import java.rmi.MarshalException;
import java.text.CollationElementIterator;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // public static double shootAlgaeDistance = 2; // m from center of field
  
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

        //Do not change these. Actual values will be calculated by the vision system.
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);

        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
        
        //Change these for fine tune vision system calculations of standard deviations.
        public static final double kXYStdDev = 0.4; 
        public static final double kThetaStdDev = 1; 

        public static final double TRANSLATION_TOLERANCE_X = 0.013; // Changed from 0.05 3/8/25
        public static final double TRANSLATION_TOLERANCE_Y = 0.013; // Changed from 0.05 3/8/25
        public static final double ROTATION_TOLERANCE = Math.toRadians(1.3); // /deg

        //Below same as pathplanner constants
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

    public static final double gyroP = 2;
    public static final double gyroI = 0.0;
    public static final double gyroD = 0.00;

    public static final String pigeonCanBus = "canivore1";


//         for (int i = 0; i < FieldConstants.Reef.branchPositions.size(); i++) {
//           for (FieldConstants.ReefHeight height : FieldConstants.ReefHeight.values()) {
//             DogLog.log("Target Pose "+ i + " " + height.toString(), FieldConstants.Reef.branchPositions.get(i).get(height).toPose2d());
          
//         }
//       }
//     }
  
}
