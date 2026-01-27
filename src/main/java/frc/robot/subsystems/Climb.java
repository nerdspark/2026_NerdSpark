// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {
  private TalonFX climbShort, climbTall, climbKicker;
  private TalonFXConfiguration climbConfig = new TalonFXConfiguration();
  private boolean ampTriggered, ampTriggerStarted = false;



//Start of Climb Sequence
//L1-------------------------
//Taller Arm goes to 30 inches and hooks on first rung and Small Arm goes to highest position
//Tall Arm pulls down until small arm then small arm hooks on first rung and Tall Arm lets go
//L2-------------------------
//Tall Arm goes to second rung position and hooks on second rung
//Small Arm unhooks from first rung
//Kicker Arm goes out and pushes robot back and allows clearance
//Tall Arm pulls down until small arm then small arm hooks on second rung and Tall Arm lets go
//L3-------------------------
//Tall Arm goes to third rung position and hooks on third rung
//Small Arm unhooks from second rung
//Kicker Arm goes out and pushes robot back and allow clearance
//Tall Arm pulls down until small arm then small arm hooks on third rung and Tall Arm lets go.
//End Climb---------------------




  /** Creates a new Climb. */
  public Climb() {
    climbTall = new TalonFX(ClimbConstants.kLeftID, ClimbConstants.canBus);
    climbShort = new TalonFX(ClimbConstants.kRightID, ClimbConstants.canBus);
    climbKicker = new TalonFX(ClimbConstants.kKickerID, ClimbConstants.canBus);
    // climbHook = new TalonFX(ClimbConstants.kHookID, ClimbConstants.canBus);
    // Initializing the motor
    climbConfig.CurrentLimits = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(ClimbConstants.climbCurrentLimit)
        .withStatorCurrentLimitEnable(true);
    climbConfig.Feedback = new FeedbackConfigs()
        .withFeedbackRotorOffset(0)
        .withSensorToMechanismRatio(1);
    climbConfig.ClosedLoopRamps = new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(ClimbConstants.rampRate);
    climbConfig.Slot0 = new Slot0Configs()
        .withKP(ClimbConstants.kP)
        .withKI(ClimbConstants.kI)
        .withKD(ClimbConstants.kD)
        .withKG(ClimbConstants.kG)
        .withKS(ClimbConstants.kS)
        .withGravityType(GravityTypeValue.Elevator_Static);

    climbTall
        .getConfigurator()
        .apply(climbConfig.withMotorOutput(new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)));
    climbShort
        .getConfigurator()
        .apply(climbConfig.withMotorOutput(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)));
    climbKicker
        .getConfigurator()
        .apply(climbConfig.withMotorOutput(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive) //Change Clockwise/CounterClockwise based on testing
            .withNeutralMode(NeutralModeValue.Brake)));
    // climbHook
    // .getConfigurator()
    // .apply(climbConfig.withMotorOutput(new MotorOutputConfigs()
    // .withInverted(InvertedValue.Clockwise_Positive)
    // .withNeutralMode(NeutralModeValue.Brake)));

    resetTallPosition();
    resetShortPosition();
    resetKickerPosition();
  }

  // public void robotAngle(Supplier<Double> position) {
  //   setClimbControl(position);
  // }

  public void setClimbTall(Supplier<Double> position) {
    climbTall.setControl(new PositionVoltage(position.get().doubleValue()));
    // climbHook.setControl(new PositionVoltage(position));

  }

  public void setClimbShort(Supplier<Double> position) {
    climbShort.setControl(new PositionVoltage(position.get().doubleValue()));
  }
  //Check if this function is needed during testing.
  public void setClimbKicker(Supplier<Double> position) {
    climbKicker.setControl(new PositionVoltage(position.get().doubleValue()));
  }

  public void resetTallPosition() {
    climbTall.setControl(new PositionVoltage(0));
  }

  public void resetShortPosition() {
    climbShort.setControl(new PositionVoltage(0));
  }

  public void resetKickerPosition() {
    climbKicker.setControl(new PositionVoltage(0));
  }

  public void setClimbTallVoltage(double voltage) {
    climbTall.setVoltage(voltage);
  }

  public void setClimbShortVoltage(double voltage) {
    climbShort.setVoltage(voltage);
  }
  public void setClimbKickerVoltage(double voltage) {
    climbKicker.setVoltage(voltage);
  }

  public boolean climbLeftAmpTriggered() {
    return Math.abs(climbTall.getStatorCurrent().getValueAsDouble()) > 10;
  }
  public boolean climbRightAmpTriggered() {
    return Math.abs(climbShort.getStatorCurrent().getValueAsDouble()) > 10;
  }
  public boolean climbKickerAmpTriggered() {
    return Math.abs(climbKicker.getStatorCurrent().getValueAsDouble()) > 10;
  }

  public double getLeftPosition() {
    return climbTall.getPosition().getValueAsDouble();
  }

  public double getRightPosition() {
    return climbShort.getPosition().getValueAsDouble();
  }

  public double getKickerPosition() {
    return climbKicker.getPosition().getValueAsDouble();
  }

  public Command tallGoToPosition(Supplier<Double> position) {
    return new InstantCommand(() -> setClimbTall(position));
  }

  public Command tallResetPosition() {
    return new InstantCommand(() -> resetTallPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("climb left position", climbTall.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("climb right position", climbShort.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("climb kicker position", climbKicker.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("climb left current (amps)", climbTall.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("climb right current (amps)", climbShort.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("climb kicker current (amps)", climbKicker.getStatorCurrent().getValueAsDouble());
  }
}
