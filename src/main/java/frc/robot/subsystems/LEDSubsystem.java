// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.nio.channels.ShutdownChannelGroupException;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import java.lang.Math;

public class LEDSubsystem extends SubsystemBase {
  private final CANdle m_candle = new CANdle(Constants.LED.id, Constants.CANbus);
  private XboxController joystick;

  // addressable LED
  // private final AddressableLED m_led = new AddressableLED(Constants.ledID);

  // private int fuelAmount; // idk how were gonna do this but we can estimate

  // private boolean fuelFull = false;
  // private boolean intakeOn = false;
  // private boolean shooterSpinning = false;
  // private boolean shooterReady = false;
  // private boolean shooterOn = false;
  // // private boolean lidClosed = false;
  // private boolean climbing = false;
  // private boolean climbDone = false;
  // private boolean visionUpdate;
  // private boolean turretLocked = false;
  // private boolean startup = false;
  // private double distance;
  private static int status = 10; // startup

  private static final RGBWColor kGreen = new RGBWColor(54, 255, 0, 0);
  // private static final RGBWColor kYellow = new RGBWColor(255, 255, 0, 0);
  private static final RGBWColor kYellow = new RGBWColor(255, 127, 0, 0); // looks like yellow
  private static final RGBWColor kRed = new RGBWColor(255, 0, 0, 0);
  private static final RGBWColor kBlack = new RGBWColor(0, 0, 0, 0);
  private static final RGBWColor kCyan = new RGBWColor(0, 160, 255, 0); // made cyan more bluey
  private static final RGBWColor kMagenta = new RGBWColor(255, 0, 255, 0);
  private static final RGBWColor kBlue = new RGBWColor(0, 0, 255, 0);
  private static final RGBWColor kWhite = new RGBWColor(255, 255, 255, 255);

  private int ledStartIndex = 8;
  private int ledEndIndex = 400;

  // private SmartDashboard smartDashboard = new SmartDashboard();

  // private boolean updated = true;

  // Creates a new LEDSubsystem
  public LEDSubsystem() {
    /*
     * 
     */

    /* Configure CANdle */
    var cfg = new CANdleConfiguration();
    /* set the LED strip type and brightness */
    cfg.LED.StripType = StripTypeValue.RGB;
    cfg.LED.BrightnessScalar = 1.0;
    /* disable status LED when being controlled */
    cfg.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

    m_candle.getConfigurator().apply(cfg);

    // // Create an LED pattern that sets the entire strip to solid red
    // LEDPattern red = LEDPattern.solid(Color.kRed);

    // // Apply the LED pattern to the data buffer
    // red.applyTo(m_ledBuffer);

    // // Write the data to the LED strip
    // m_led.setData(m_ledBuffer);

    // SmartDashboard.putData("LED state", (Sendable) m_candle.getAppliedControl());

    CommandXboxController joystick = new CommandXboxController(Constants.LED.testJoystickID);

  }

  public CANdle getCandle() {
    return m_candle;
  }

  public void solidColor(RGBWColor color) {
    // m_candle.setControl(new StrobeAnimation(ledStartIndex, ledEndIndex)
    // .withSlot(Constants.ledBlinkID)
    // .withColor(kBlack)
    // .withFrameRate(0));

    // empty(Constants.ledBlinkID);
    // empty(Constants.ledPulseID);
    // empty(Constants.ledRainbowID);
    empty();
    m_candle.setControl(new SolidColor(ledStartIndex, ledEndIndex).withColor(color));
  }

  public void solidColor(RGBWColor color, double brightness) {
    empty();
    m_candle.setControl(new SolidColor(ledStartIndex, ledEndIndex)
        .withColor(color.scaleBrightness(brightness)));
  }

  public void blinkColor(RGBWColor color) {
    // m_candle.setControl(new EmptyAnimation(Constants.ledID));
    // m_candle.se
    // m_candle.setControl(new SolidColor(ledStartIndex,
    // ledEndIndex).withColor(color));

    // empty();
    empty(Constants.LED.solidID);
    empty(Constants.LED.pulseID);
    empty(Constants.LED.rainbowID);
    m_candle.setControl(
        new StrobeAnimation(ledStartIndex, ledEndIndex)
            .withSlot(Constants.LED.blinkID)
            .withColor(color)
            .withFrameRate(Constants.LED.framerate));
    // .withUpdateFreqHz(60));
  }

  public void flowColor(RGBWColor color) {
    ControlRequest previousControl = m_candle.getAppliedControl();
    // m_candle.setControl(new EmptyAnimation(Constants.ledID));
    // if ((int) ( / (time * 1000)) % 2 == 0) {
    // empty();
    // m_candle.setControl(
    // new SolidColor(ledStartIndex, ledEndIndex)
    // .withColor(color));
    // } else

    // empty();
    empty(Constants.LED.solidID);
    empty(Constants.LED.blinkID);
    empty(Constants.LED.rainbowID);
    m_candle
        .setControl(new ColorFlowAnimation(ledStartIndex, ledEndIndex)
            .withSlot(Constants.LED.pulseID)
            .withColor(color));

    m_candle.setControl(previousControl);
  }

  public void empty(int slot) {
    m_candle.setControl(new EmptyAnimation(slot));
  }

  public void empty() {
    m_candle.setControl(new EmptyAnimation(Constants.LED.solidID));
    m_candle.setControl(new EmptyAnimation(Constants.LED.blinkID));
    m_candle.setControl(new EmptyAnimation(Constants.LED.rainbowID));
    m_candle.setControl(new EmptyAnimation(Constants.LED.pulseID));
  }

  public void rainbow() {
    // empty();
    empty(Constants.LED.solidID);
    empty(Constants.LED.pulseID);
    empty(Constants.LED.blinkID);
    m_candle.setControl(new RainbowAnimation(ledStartIndex, ledEndIndex)
        .withSlot(Constants.LED.rainbowID)
        .withFrameRate(Constants.LED.framerate));
  }

  public CANdle getM_candle() {
    return m_candle;
  }

  public XboxController getJoystick() {
    return joystick;
  }

  public void setJoystick(XboxController joystick) {
    this.joystick = joystick;
  }

  // public boolean isFuelFull() {
  //   return fuelFull;
  // }

  // public void setFuelFull(boolean fuelFull) {
  //   this.fuelFull = fuelFull;
  // }

  // public boolean isIntakeOn() {
  //   return intakeOn;
  // }

  // public void setIntakeOn(boolean intakeOn) {
  //   this.intakeOn = intakeOn;
  // }

  // public boolean isShooterSpinning() {
  //   return shooterSpinning;
  // }

  // public void setShooterSpinning(boolean shooterSpinning) {
  //   this.shooterSpinning = shooterSpinning;
  // }

  // public boolean isShooterReady() {
  //   return shooterReady;
  // }

  // public void setShooterReady(boolean shooterReady) {
  //   this.shooterReady = shooterReady;
  // }

  // public boolean isShooterOn() {
  //   return shooterOn;
  // }

  // public void setShooterOn(boolean shooterOn) {
  //   this.shooterOn = shooterOn;
  // }

  // public boolean isClimbing() {
  //   return climbing;
  // }

  // public void setClimbing(boolean climbing) {
  //   this.climbing = climbing;
  // }

  // public boolean isClimbDone() {
  //   return climbDone;
  // }

  // public void setClimbDone(boolean climbDone) {
  //   this.climbDone = climbDone;
  // }

  public int getLedStartIndex() {
    return ledStartIndex;
  }

  public void setLedStartIndex(int ledStartIndex) {
    this.ledStartIndex = ledStartIndex;
  }

  public int getLedEndIndex() {
    return ledEndIndex;
  }

  public void setLedEndIndex(int ledEndIndex) {
    this.ledEndIndex = ledEndIndex;
  }

  // public boolean isVisionUpdate() {
  //   return visionUpdate;
  // }

  // public void setVisionUpdate(boolean visionUpdate) {
  //   this.visionUpdate = visionUpdate;
  // }

  // public boolean isTurretLocked() {
  //   return turretLocked;
  // }

  // public void setTurretLocked(boolean turretLocked) {
  //   this.turretLocked = turretLocked;
  // }

  // public double getDistance() {
  //   return distance;
  // }

  // public void setDistance(double distance) {
  //   this.distance = distance;
  // }


  
  private double distanceCurve(double distance) { 
    return Math.pow(Math.abs(distance), 2);
  }

  public int getStatus() {
    return status;
  }

  public void setStatus(int status) {
    this.status = status;
    updateLED();
  }

  public void updateLED() {   
    // TODO flash red for no apriltags detected
    // TODO lined up for climb. maye change magenta to that
    // red error state
    // if (climbing) { // climbing
    //   blinkColor(kMagenta);
    // } else if (climbDone) {
    //   rainbow();
    // } else if (intakeOn) { // intaking

    //   if (fuelFull) {
    //     solidColor(kBlue);
    //   } else {
    //     blinkColor(kYellow);
    //   }

    // } else if (shooterOn) { // shooting
    //   blinkColor(kCyan);

    // } else if (shooterReady && turretLocked) { // shooter ready
    //   solidColor(kGreen, distanceCurve(distance));

    // } else if (shooterReady || shooterSpinning) { // shooter spinning
    //   blinkColor(kGreen);

    // } else if (startup) {
    //   rainbow();
    // }    if (visionUpdate) { // vision updating
    //   // System.currentTimeMillis()
    //   flowColor(kWhite);
    // }

    switch(status) { // TODO implement override logic
      // maybe do that with ints and status variables in Constants and comparing if they are greater
      
      case Constants.LED.shooting:
        solidColor(kGreen);
        break;
      case Constants.LED.readyToShoot:
        blinkColor(kGreen);
        break;
      case Constants.LED.intaking:
        solidColor(kBlue);
        break;
      case Constants.LED.aiming:
        solidColor(kCyan);
        break;
      case Constants.LED.noAprilTags:
        blinkColor(kRed);
        break;
      case Constants.LED.climbReady: //lined up for climb
        solidColor(kMagenta); 
        break;
      case Constants.LED.idle:
        solidColor(kWhite);
        break;
      case Constants.LED.intakeDeployed:
        blinkColor(kBlue);
        break;
      case Constants.LED.safe:
        solidColor(kYellow);
        break;
      case Constants.LED.startup:
        rainbow();
        break;
      case Constants.LED.reset:
        // empty();
        solidColor(kBlack);
        break;
      default:
        solidColor(kBlack);
        break;
    }
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    // updateLED();

    // climbing

  }

}
