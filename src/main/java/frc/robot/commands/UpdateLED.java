// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LEDSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class UpdateLED extends Command {
  /** Creates a new UpdateLED. */
  private boolean fuelFull = false;
  private boolean intakeOn = false;
  private boolean shooterSpinning = false;
  private boolean shooterReady = false;
  private boolean shooterOn = false;
  // private boolean lidClosed = false;
  private boolean climbing = false;
  private boolean climbDone = false;

  private static final RGBWColor kGreen = new RGBWColor(0, 255, 0, 0);
  private static final RGBWColor kYellow = new RGBWColor(255, 255, 0, 0);
  private static final RGBWColor kRed = new RGBWColor(255, 0, 0, 0);
  private static final RGBWColor kBlack = new RGBWColor(0, 0, 0, 0);
  private static final RGBWColor kCyan = new RGBWColor(0, 255, 255, 0);
  private static final RGBWColor kMagenta = new RGBWColor(255, 0, 255, 0);
  private static final RGBWColor kBlue = new RGBWColor(0, 0, 255, 0);
  private static final RGBWColor kWhite = new RGBWColor(0, 0, 0, 255);

  LEDSubsystem led = new LEDSubsystem();
  // CommandXboxController joystick = RobotContainer.joystick;

  Supplier<Boolean> aSupplier;
  Supplier<Boolean> bSupplier;
  Supplier<Boolean> xSupplier;
  Supplier<Boolean> ySupplier;
  Supplier<Boolean> upSupplier;
  Supplier<Boolean> downSupplier;
  Supplier<Boolean> leftSupplier;
  Supplier<Boolean> rightSupplier;

  public UpdateLED(LEDSubsystem ledSubsystem,
      Supplier<Boolean> aSupplier, Supplier<Boolean> bSupplier, Supplier<Boolean> xSupplier,
      Supplier<Boolean> ySupplier, Supplier<Boolean> upSupplier, Supplier<Boolean> downSupplier,
      Supplier<Boolean> leftSupplier, Supplier<Boolean> rightSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.aSupplier = aSupplier;
    this.bSupplier = bSupplier;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.upSupplier = upSupplier;
    this.downSupplier = downSupplier;
    this.leftSupplier = leftSupplier;
    this.rightSupplier = rightSupplier;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    fuelFull = leftSupplier.get();
    intakeOn = bSupplier.get();
    shooterSpinning = xSupplier.get();
    shooterReady = ySupplier.get();
    shooterOn = upSupplier.get();
    // private boolean lidClosed = false;
    climbing = downSupplier.get();
    climbDone = rightSupplier.get();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (climbing) { // climbing
      led.blinkColor(kMagenta);

    } else if (climbDone) {
      led.solidColor(kMagenta);
    } else if (intakeOn) { // intaking

      if (fuelFull) {
        led.solidColor(kBlue);
      } else {
        led.blinkColor(kYellow);
      }

    } else if (shooterOn) { // shooting
      led.blinkColor(kCyan);

    } else if (shooterReady) { // shooter ready
      led.solidColor(kGreen);
    } else if (shooterSpinning) { // shooter spinning
      led.blinkColor(kGreen);

    } else if (!fuelFull && !intakeOn && !shooterOn && !shooterSpinning) {
      led.solidColor(kRed);
    } else {

      led.rainbow();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
