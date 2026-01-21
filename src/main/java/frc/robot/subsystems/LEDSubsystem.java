// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
  private final CANdle m_candle = new CANdle(Constants.ledID, "rio");
  private XboxController joystick;

  //addressable LED
  // private final AddressableLED m_led = new AddressableLED(Constants.ledID);
  
 
  private boolean intakeOn = false;
    private boolean shooterOn = false;
    private boolean lidClosed = false;
    private boolean climbing = false;

    private RGBWColor red = new RGBWColor(255, 0, 0);
    private Color yellow = new Color(255, 255, 0);
    private Color green = new Color(0, 255, 0);
    private Color cyan = new Color(0, 255, 255);
    private Color blue = new Color(0, 0, 255);
    private Color magenta = new Color(255, 0, 255);
    private Color white = new Color(255, 255, 255);
    private Color black = new Color(0, 0, 0);

   /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    // actions: ready to aim - shooter angle spun up
    // align for climb -/ distance to climb ig
    // storage lid open/closed (storage full)
    // shooter spinning
    // distance to hub (?)
    // blind opponent drivers :DD

    //TODO find the method to set color
    m_candle.   m_candle.setControl(new SolidColor(8, Constants.totalLEDs-1));
    m_candle.setControl(new ColorFlowAnimation(0, 0));
    m_candle.setControl(new StrobeAnimation(0, Constants.totalLEDs-1));

    AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(Constants.totalLEDs);
    m_led.setLength(m_ledBuffer.getLength());

    m_led.setData(m_ledBuffer);
    m_led.start();



// Create an LED pattern that sets the entire strip to solid red
LEDPattern red = LEDPattern.solid(Color.kRed);

// Apply the LED pattern to the data buffer
red.applyTo(m_ledBuffer);

// Write the data to the LED strip
m_led.setData(m_ledBuffer);

    }

    

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    if(intakeOn && !shooterOn) {

    }
  }
}
