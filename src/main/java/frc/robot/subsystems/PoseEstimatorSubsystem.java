package frc.robot.subsystems;

import static frc.robot.Constants.Vision.USE_VISION;
import static frc.robot.Constants.Vision.kCameraNameBack;
import static frc.robot.Constants.Vision.kCameraNameFront;
import static frc.robot.Constants.Vision.kRobotToCamBack;
import static frc.robot.Constants.Vision.kRobotToCamFront;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;

import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;


public class PoseEstimatorSubsystem extends SubsystemBase {

    private final CommandSwerveDrivetrain driveTrain;
    public Vision visionFront;
    public Vision visionBack;
    // private static Notifier allNotifier;

    Pose2d robotPose2d = new Pose2d();
    StructPublisher<Pose2d> publisher;


    //private final Pigeon2 gyro = new Pigeon2(TunerConstants.kPigeonId);
    //private PIDController GyroPID = new PIDController(Constants.gyroP, Constants.gyroI, Constants.gyroD);
    //public double targetAngle = 0;
    //private Rotation2d gyroResetAngle = new Rotation2d();

       
    private Field2d field = new Field2d(); 
          
        // Simulation
    
        public PoseEstimatorSubsystem(CommandSwerveDrivetrain driveTrain) {
            this.driveTrain = driveTrain;

            this.visionBack = new Vision(driveTrain::addVisionMeasurement, kCameraNameBack, kRobotToCamBack);
            this.visionFront = new Vision(driveTrain::addVisionMeasurement, kCameraNameFront, kRobotToCamFront);
    }

    @Override
    public void periodic() {
        // Update pose estimator with drivetrain sensors
        if(USE_VISION) {
            visionFront.periodic();
            visionBack.periodic();
        }

        if (getCurrentPose() != null) {
            field.setRobotPose(getCurrentPose());
            robotPose2d = getCurrentPose();
            publisher.set(robotPose2d);

            // field.getObject("VisionEstimation").setPoses();

            SmartDashboard.putData("Robot Pose in Field", field);
            SmartDashboard.putString("Formatted Pose", getFomattedPose());

            // DogLog.log("PoseEstimator/Pose", getCurrentPose());
            // DogLog.log("PoseEstimator/Formatted Pose", getFomattedPose());            

        }
    }
    

    private String getFomattedPose() {
        var pose = getCurrentPose();
        return String.format(
                "(%.3f, %.3f) %.2f degrees",
                pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    }

    private String getFomattedPose(Pose2d pose) {
        return String.format(
                "(%.3f, %.3f) %.2f degrees",
                pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    }

    public Pose2d getCurrentPose() {
        return driveTrain.getState().Pose;
    }

    /**
     * Resets the current pose to the specified pose. This should ONLY be called
     * when the robot's position on the field is known, like at the beginning of
     * a match.
     *
     * @param newPose new pose
     */
    public void setCurrentPose(Pose2d newPose) {
        //driveTrain.seedFieldRelative(newPose);
        driveTrain.resetPose(newPose);
    }

    /**
     * Resets the position on the field to 0,0 0-degrees, with forward being
     * downfield. This resets
     * what "forward" is for field oriented driving.
     */
    public void resetFieldPosition() {
        setCurrentPose(new Pose2d());
    }

    public void printMatrixValues(Matrix<N3, N1> curStdDevs) {
        
        for (int i = 0; i < curStdDevs.getNumRows(); i++) {
            for (int j = 0; j < curStdDevs.getNumCols(); j++) {
                DogLog.log("Stddev "+ i,curStdDevs.get(i, j));
            }
        }
    }
}