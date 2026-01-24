// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.nio.channels.ShutdownChannelGroupException;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
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

public class LEDSubsystem extends SubsystemBase {
  private final CANdle m_candle = new CANdle(Constants.ledID, "rio");
  private XboxController joystick;

  // addressable LED
  // private final AddressableLED m_led = new AddressableLED(Constants.ledID);

  // private int fuelAmount; // idk how were gonna do this but we can estimate
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

  // private boolean updated = true;

  // Creates a new LEDSubsystem
  public LEDSubsystem() {
    /*
     * Actions
     * Empty: Red
     * Intaking: Blinking Yellow
     * Not intaking, but not holding optimal amount of fuel: Blinking Blue
     * Full: Blue
     * Spinning up shooter: Blinking green
     * Ready to shoot: Green
     * Shooting: Blinking cyan
     * Climbing: Blinking Magenta
     * Climbed: Magenta
     */

    /* Configure CANdle */
    var cfg = new CANdleConfiguration();
    /* set the LED strip type and brightness */
    cfg.LED.StripType = StripTypeValue.GRB;
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

    SmartDashboard.putData("LED state", (Sendable)m_candle.getAppliedControl());

    CommandXboxController joystick = new CommandXboxController(Constants.testJoystickID);

  }

  public CANdle getCandle() {
    return m_candle;
  }

  public void solidColor(RGBWColor color) {
    m_candle.setControl(new SolidColor(0, 7).withColor(color));
  }

  public void blinkColor(RGBWColor color) {
    m_candle.setControl(
        new StrobeAnimation(0, 7)
            .withSlot(Constants.ledID)
            .withColor(color)
            .withFrameRate(Constants.ledFramerate));
  }

  public void rainbow() {
    m_candle.setControl(new RainbowAnimation(0, 7)
        .withSlot(Constants.ledID)
        .withFrameRate(Constants.ledFramerate));
  }

  // public void updateLED() {
  //     if (climbing) { // climbing
  //       m_candle.setControl(
  //           new StrobeAnimation(0, 7)
  //               .withSlot(Constants.ledID)
  //               .withColor(kMagenta)
  //               .withFrameRate(Constants.ledFramerate));

  //     } else if (climbDone) {
  //       m_candle.setControl(new SolidColor(0, 7).withColor(kMagenta));

  //     } else if (intakeOn) { // intaking

  //       if (fuelFull) {
  //         m_candle.setControl(new SolidColor(0, 7).withColor(kBlue));

  //       } else {
  //         m_candle.setControl(
  //             new StrobeAnimation(0, 7)
  //                 .withSlot(Constants.ledID)
  //                 .withColor(kYellow)
  //                 .withFrameRate(Constants.ledFramerate));
  //       }

  //     } else if (shooterOn) { // shooting
  //       m_candle.setControl(
  //           new StrobeAnimation(0, 7)
  //               .withSlot(Constants.ledID)
  //               .withColor(kCyan)
  //               .withFrameRate(Constants.ledFramerate));

  //     } else if (shooterReady) { // shooter ready
  //       m_candle.setControl(new SolidColor(0, 7).withColor(kGreen));

  //     } else if (shooterSpinning) { // shooter spinning
  //       m_candle.setControl(
  //           new StrobeAnimation(0, 7)
  //               .withSlot(Constants.ledID)
  //               .withColor(kGreen)
  //               .withFrameRate(Constants.ledFramerate));

  //     } else if (!fuelFull && !intakeOn && !shooterOn && !shooterSpinning) {
  //       m_candle.setControl(new SolidColor(0, 7).withColor(kRed));

  //     } else {
  //       m_candle.setControl(new RainbowAnimation(0, 7)
  //           .withSlot(Constants.ledID)
  //           .withFrameRate(Constants.ledFramerate));
  //     }

    
  // }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    // updateLED();

    // climbing

  }
}
