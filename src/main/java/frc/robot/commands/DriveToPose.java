// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import dev.doglog.DogLog;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.Constants.Vision;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.AllianceFlipUtil;

public class DriveToPose extends Command {
  private final CommandSwerveDrivetrain drive;
  private final Supplier<Pose2d> poseSupplier;

  private boolean running = false;
  double loopPeriodSecs = 0.02;
//   private final ProfiledPIDController driveController =
//       new ProfiledPIDController(
//           Constants.Vision.kPXController, Constants.Vision.kIXController, Constants.Vision.kDXController, new TrapezoidProfile.Constraints(Constants.Vision.MAX_VELOCITY,Constants.Vision.MAX_ACCELARATION), loopPeriodSecs);
//   private final ProfiledPIDController thetaController =
//       new ProfiledPIDController(
//           Constants.Vision.kPXController, Constants.Vision.kIThetaController, Constants.Vision.kDThetaController, new TrapezoidProfile.Constraints(Math.toRadians(Constants.Vision.MAX_VELOCITY_ROTATION), Math.toRadians(Constants.Vision.MAX_ACCELARATION_ROTATION)), loopPeriodSecs);
  
  private final SlewRateLimiter driveLimiter = new SlewRateLimiter(Constants.Vision.MAX_ACCELERATION);
  private final ProfiledPIDController driveController =
      new ProfiledPIDController(
          Constants.Vision.kPXController, Constants.Vision.kIXController, Constants.Vision.kDXController, new TrapezoidProfile.Constraints(Constants.Vision.MAX_VELOCITY,Constants.Vision.MAX_ACCELERATION), loopPeriodSecs); //10, 0, 0
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          Constants.Vision.kPThetaController, Constants.Vision.kIThetaController, Constants.Vision.kDThetaController, new TrapezoidProfile.Constraints(Math.toRadians(Constants.Vision.MAX_VELOCITY_ROTATION), Math.toRadians(Constants.Vision.MAX_ACCELARATION_ROTATION)), loopPeriodSecs); //3, 10, 0
  private double driveErrorAbs;
  private double thetaErrorAbs;
  private Translation2d lastSetpointTranslation;
  private final SwerveRequest.ApplyRobotSpeeds driveToPoseRequest = new SwerveRequest.ApplyRobotSpeeds();
  private Supplier<Translation2d> linearFF = () -> Translation2d.kZero;
  private DoubleSupplier omegaFF = () -> 0.0;

  /** Drives to the specified pose under full software control. */
  public DriveToPose(CommandSwerveDrivetrain drive, Supplier<Pose2d> poseSupplier) {
    this.drive = drive;
    this.poseSupplier = poseSupplier;
    addRequirements(drive);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

    /** Drives to the specified pose under full software control. */
    public DriveToPose(CommandSwerveDrivetrain drive, Supplier<Pose2d> poseSupplier,
     Supplier<Translation2d> linearFF,
      DoubleSupplier omegaFF) {
      this.drive = drive;
      this.poseSupplier = poseSupplier;
      this.linearFF = linearFF;
      this.omegaFF = omegaFF;
      addRequirements(drive);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }
  
  
  @Override
  public void initialize() {

    driveController.setTolerance(Constants.Vision.TRANSLATION_TOLERANCE_X, Constants.Vision.VELOCITY_TOLERANCE_X);
    thetaController.setTolerance(Constants.Vision.ROTATION_TOLERANCE,Constants.Vision.VELOCITY_TOLERANCE_OMEGA);
    // Reset all controllers
    var currentPose = drive.getState().Pose;
    
    driveController.reset(
        currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation()),
        // Math.min(
            // 0.0,
            -new Translation2d(
                drive.getCurrentRobotChassisSpeeds().vxMetersPerSecond, drive.getCurrentRobotChassisSpeeds().vyMetersPerSecond).getNorm());
                // .rotateBy(
                //     poseSupplier
                //         .get()
                //         .getTranslation()
                //         .minus(drive.getState().Pose.getTranslation())
                //         .getAngle())
                //         .unaryMinus()
                // .getX());
    thetaController.reset(currentPose.getRotation().getRadians(), drive.getCurrentRobotChassisSpeeds().omegaRadiansPerSecond);
    lastSetpointTranslation = drive.getState().Pose.getTranslation();
  }

  @Override
  public void execute() {
    running = true;

    // Get current and target pose
    var currentPose = drive.getState().Pose;
    var targetPose = poseSupplier.get();
    Transform2d error = currentPose.minus(targetPose);
        SignalLogger.writeDouble("X", error.getX());
        SignalLogger.writeDouble("Y", error.getY());
        SignalLogger.writeDouble("O", error.getRotation().getDegrees());

        System.out.println("x: " + error.getX() + "; Y: " + error.getY() + "; O: " + error.getRotation().getDegrees());
        
    // Calculate drive speed
    double currentDistance =
        currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation());
    double ffScaler =
        0.0;
    driveErrorAbs = currentDistance;

    driveController.reset(
        lastSetpointTranslation.getDistance(targetPose.getTranslation()),
        driveController.getSetpoint().velocity);

    double driveVelocityScalar = driveLimiter.calculate(
        // driveController.getSetpoint().velocity * ffScaler +
             driveController.calculate(driveErrorAbs, 0.0));

    if (currentDistance < driveController.getPositionTolerance()) {driveVelocityScalar = 0.0;}
    lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(
                new Transform2d(new Translation2d(driveController.getSetpoint().position, 0.0), new Rotation2d()))
            .getTranslation();

    // Calculate theta speed
    double thetaVelocity =
       thetaController.getSetpoint().velocity * ffScaler +
         thetaController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    thetaErrorAbs =
        Math.abs((currentPose.getRotation().minus(targetPose.getRotation())).getRadians());
    if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;


    Translation2d driveVelocity =
        new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(new Transform2d(driveVelocityScalar, 0.0, new Rotation2d()))
            .getTranslation();

    // Scale feedback velocities by input ff
    final double linearS = linearFF.get().getNorm() * 3.0;
    final double thetaS = Math.abs(omegaFF.getAsDouble()) * 3.0;
    if (Vision.DOGLOG_ENABLED){
    DogLog.log("DriveToPose/driveVelocity.X", driveVelocity.getX());
    DogLog.log("DriveToPose/driveVelocity.Y", driveVelocity.getY());
    DogLog.log("DriveToPose/thetaVelocity", thetaVelocity);
    DogLog.log("DriveToPose/linearS", linearS);
    DogLog.log("DriveToPose/thetaS", thetaS);
    }


    // if(linearS >0)
    // driveVelocity =
    //     driveVelocity.interpolate(linearFF.get().times(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)), linearS);
    // if(thetaS >0 )
    // thetaVelocity =
    //     MathUtil.interpolate(
    //         thetaVelocity, omegaFF.getAsDouble() * RotationsPerSecond.of(0.75).in(RadiansPerSecond), thetaS);
    //         if (Vision.DOGLOG_ENABLED){

    //         DogLog.log("DriveToPose/driveVelocity.X_I", driveVelocity.getX());
    //         DogLog.log("DriveToPose/driveVelocity.Y_I", driveVelocity.getY());
    //         DogLog.log("DriveToPose/thetaVelocity_I", thetaVelocity);
    //         }
    
    drive.setControl(driveToPoseRequest.withSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(driveVelocity.getX(),driveVelocity.getY(),
    thetaVelocity, drive.getState().Pose.getRotation())).withDriveRequestType(DriveRequestType.Velocity));
    // Log data
    if (Vision.DOGLOG_ENABLED){

    DogLog.log("DriveToPose/DistanceMeasured", currentDistance);
    DogLog.log("DriveToPose/DistanceSetpoint", driveController.getSetpoint().position);
    DogLog.log("DriveToPose/ThetaMeasured", Math.toDegrees(currentPose.getRotation().getRadians()));
    DogLog.log("DriveToPose/ThetaSetpoint", Math.toDegrees(thetaController.getSetpoint().position));
    DogLog.log(
            "DriveToPose/DriveToPoseSetpoint",
            new Pose2d(
                lastSetpointTranslation, new Rotation2d(thetaController.getSetpoint().position)));
    DogLog.log("DriveToPose/DriveToPoseGoal", targetPose);
    DogLog.log("DriveToPose/ThetaErrorAbs", thetaErrorAbs);
    DogLog.log("DriveToPose/DriveErrorAbs", driveErrorAbs);
    DogLog.log("DriveToPose/DriveTolerance", driveController.getPositionTolerance());
    DogLog.log("DriveToPose/DriveVelocityTolerance", driveController.getVelocityTolerance());
    DogLog.log("DriveToPose/DriveVelocityError", driveController.getVelocityError());
    DogLog.log("DriveToPose/ThetaVelocityTolerance", thetaController.getVelocityTolerance());
    DogLog.log("DriveToPose/ThetaVelocityError", thetaController.getVelocityError());

    DogLog.log("DriveToPose/DriveControllerAtGoal", driveController.atGoal());
    DogLog.log("DriveToPose/ThetaControllerAtGoal", thetaController.atGoal());
    }
  }

  @Override
  public void end(boolean interrupted) {
    running = false;
    drive.applyRequest(() -> new SwerveRequest.SwerveDriveBrake());
    if (Vision.DOGLOG_ENABLED){

    DogLog.log("DriveToPose/DriveToPoseSetpoint", new double[] {});
    DogLog.log("DriveToPose/DriveToPoseGoal", new double[] {});
    }
  }

  /** Checks if the robot is stopped at the final pose. */
  public boolean atGoal() {
    if (Vision.DOGLOG_ENABLED){

    DogLog.log("DriveToPose/DriveControllerAtGoal1", driveController.atGoal());
    DogLog.log("DriveToPose/ThetaControllerAtGoal1", thetaController.atGoal());
    }
    return running && driveController.atGoal() && thetaController.atGoal();
  }

  /** Checks if the robot pose is within the allowed drive and theta tolerances. */
  public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
    return running
        && Math.abs(driveErrorAbs) < driveTolerance
        && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
  }

  /** Returns whether the command is actively running. */
  public boolean isRunning() {
    return running;
  }

      // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.atGoal();
    }

  
    
}