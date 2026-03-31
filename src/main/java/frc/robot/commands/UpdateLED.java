// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import com.ctre.phoenix6.controls.RainbowAnimation;
// import com.ctre.phoenix6.controls.SolidColor;
// import com.ctre.phoenix6.controls.StrobeAnimation;
// import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.Constants.Vision.VisionStatus;
// import frc.robot.RobotContainer;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class UpdateLED extends Command {
  /** Creates a new UpdateLED. */

  // private boolean readyToShoot  ;
  // private boolean shooting  ;
  // private boolean intaking  ;
  // private boolean aiming  ;
  // private boolean noAprilTags ;
  // private boolean climbReady ;
  // private boolean idle;
  // private boolean intakeDeployed;
  // private boolean safe;
  // private boolean startup;

  // private int pastStatus;
  
  private int status;


  LEDSubsystem led;
  PoseEstimatorSubsystem poseEstimator;
  Turret turret;
  // CommandXboxController joystick = RobotContainer.joystick;

  // Supplier<Boolean> aSupplier;
  // Supplier<Boolean> bSupplier;
  // Supplier<Boolean> xSupplier;
  // Supplier<Boolean> ySupplier;
  // Supplier<Boolean> upSupplier;
  // Supplier<Boolean> downSupplier;
  // Supplier<Boolean> leftSupplier;
  // Supplier<Boolean> rightSupplier;
  // Supplier<Boolean> leftBSupplier;
  // Supplier<Boolean> rightBSupplier;

 private boolean blUpdated;
 private boolean brUpdated;
 private boolean flUpdated;
 private boolean frUpdated;
private double hubDistance;
private double turretX;
private double turretY;
private VisionStatus visionStatus;

  
  public UpdateLED(LEDSubsystem ledSubsystem, PoseEstimatorSubsystem poseEstimator, Turret turret //,
  // Supplier<Integer> statusSupplier
    // Supplier<Boolean> aSupplier, Supplier<Boolean> bSupplier, Supplier<Boolean> xSupplier,
    // Supplier<Boolean> ySupplier, Supplier<Boolean> upSupplier, Supplier<Boolean> downSupplier,
    // Supplier<Boolean> leftSupplier, Supplier<Boolean> rightSupplier, Supplier<Boolean> leftBSupplier, Supplier<Boolean> rightBSupplier//, Supplier<Double> leftStickSupplier
    ) {

      addRequirements(ledSubsystem);
      // Use addRequirements() here to declare subsystem dependencies.
      // this.aSupplier = aSupplier;
      // this.bSupplier = bSupplier;
      // this.xSupplier = xSupplier;
      // this.ySupplier = ySupplier;
      // this.upSupplier = upSupplier;
      // this.downSupplier = downSupplier;
      // this.leftSupplier = leftSupplier;
      // this.rightSupplier = rightSupplier;
      // this.leftBSupplier = leftBSupplier;
      // this.rightBSupplier = rightBSupplier;

      this.led = ledSubsystem;
      this.poseEstimator = poseEstimator;
      this.turret = turret;


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // readyToShoot = leftSupplier.get();
    // shooting = bSupplier.get();
    // intaking = xSupplier.get();
    // aiming = ySupplier.get();
    // noAprilTags = upSupplier.get();
    // // private boolean lidClosed = false;
    // climbReady = downSupplier.get();
    // idle = rightSupplier.get();
    // intakeDeployed = aSupplier.get();
    // // status = statusSupplier.get();
    // safe = leftBSupplier.get();
    // startup = rightBSupplier.get();
    status = Constants.LED.startup;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // shooting = leftSupplier.get();
    // readyToShoot = bSupplier.get();
    // intaking = xSupplier.get();
    // aiming = ySupplier.get();
    // noAprilTags = upSupplier.get();
    // // private boolean lidClosed = false;
    // climbReady = downSupplier.get();
    // intakeDeployed = rightSupplier.get();
    // safe = aSupplier.get();
    // // status = statusSupplier.get();
    // idle = leftBSupplier.get();
    // startup = rightBSupplier.get();

    

    // if(noAprilTags) {
    //   status = Constants.LED.noAprilTags;
    // } else if(safe) {
    //   status = Constants.LED.safe;
    // } else if(shooting) {
    //   status = Constants.LED.shooting;
    // } else if(aiming) {
    //   status = Constants.LED.aiming;
    // } else if(readyToShoot) {
    //   status = Constants.LED.readyToShoot;
    // } else if(climbReady) {
    //   status = Constants.LED.climbReady;
    // } else if(intakeDeployed) {
    //   status = Constants.LED.intakeDeployed;
    // } else if(intaking) {
    //   status = Constants.LED.intaking;
    // } else if(idle) {
    //   status = Constants.LED.idle;
    // } else if(startup) {
    //   status = Constants.LED.startup;
    // } else {
    //   status = 99;
    // }

    // // UPDATED RECENTLY
    // blUpdated = SmartDashboard.getBoolean("BL Updated Recently", false);
    // brUpdated = SmartDashboard.getBoolean("BR Updated Recently", false);
    // flUpdated = SmartDashboard.getBoolean("FL Updated Recently", false);
    // frUpdated = SmartDashboard.getBoolean("FR Updated Recently", false);

    // if (blUpdated && brUpdated || blUpdated && frUpdated || blUpdated && flUpdated 
    // || brUpdated && flUpdated || brUpdated && frUpdated || flUpdated && frUpdated) {
	  //   status = Constants.LED.shooting;
    // } else if(blUpdated || brUpdated || flUpdated || frUpdated) {
    //   status = Constants.LED.readyToShoot;
    // } else {
    //   status = Constants.LED.noAprilTags;
    // }

    visionStatus = poseEstimator.getOverallVisionStatus(); // VISION STATUS

    switch(visionStatus) {
      case BEST:
      status = Constants.LED.visionBest;
      break;
      case OK:
      status = Constants.LED.visionOk;
      break;
      case BAD:
      status = Constants.LED.visionBad;
      break;
    }

    turretX = turret.getTurretPose().getX(); //poseEstimator.getCurrentPose().getX();
    turretY = turret.getTurretPose().getY(); //poseEstimator.getCurrentPose().getY();
    hubDistance = Math.hypot(
    turretX - FieldConstants.Hub.topCenterPoint.getX(),
    turretY - FieldConstants.Hub.topCenterPoint.getY());
    
    if(hubDistance < Units.feetToMeters(Constants.LED.hubDistanceLimitFeet)) {
      status = Constants.LED.closeToBub;
    }

    led.setStatus(status);

    // System.out.println(status);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    led.setStatus(99);
    // led.solidColor(kBlack);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (pastStatus != "reset" && status == "reset") {
    //   return true;//!aSupplier.get();
    // } else {
    //   return pastStatus == status;
    // }
    return false;//pastStatus == status;
    

  }
}
