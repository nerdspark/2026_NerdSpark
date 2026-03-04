// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// TODO: Change direction of motors

package frc.robot.subsystems.Climb;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class LeftClimb extends SubsystemBase {
  private TalonFX climbLeft, climbRight;
  private TalonFXConfiguration climbConfig = new TalonFXConfiguration();
  private boolean ampTriggered, ampTriggerStarted = false;
  private boolean isClimbing = false;
  // private final TalonFXSimState tallSim;
  // private final TalonFXSimState shortSim;

  private MotionMagicConfigs motionMagicConfigs = climbConfig.MotionMagic;
  final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

  /** Creates a new LeftClimb. */
  public LeftClimb() {
    climbLeft = new TalonFX(ClimbConstants.kLeftID, ClimbConstants.canBus);

    // Initializing the motor
    climbConfig.CurrentLimits = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(ClimbConstants.climbCurrentLimit)
        .withStatorCurrentLimitEnable(true);
    climbConfig.Feedback = new FeedbackConfigs()
        .withFeedbackRotorOffset(0)
        .withSensorToMechanismRatio(ClimbConstants.sensorToMechanismRatio);
    // climbConfig.ClosedLoopRamps = new
    // ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(ClimbConstants.rampRate);
    climbConfig.Slot0 = new Slot0Configs()
        .withKP(ClimbConstants.kP)
        .withKI(ClimbConstants.kI)
        .withKD(ClimbConstants.kD)
        .withKG(ClimbConstants.kG)
        .withKS(ClimbConstants.kS)
        .withKA(ClimbConstants.kA)
        .withKV(ClimbConstants.kV)
        .withGravityType(GravityTypeValue.Elevator_Static);

    // motionMagicConfigs.MotionMagicCruiseVelocity =
    // ClimbConstants.motionMagicCruiseVelocity;
    // motionMagicConfigs.MotionMagicAcceleration =
    // ClimbConstants.motionMagicAcceleration;
    // motionMagicConfigs.MotionMagicJerk = ClimbConstants.motionMagicJerk;

    configMotionMagic(ClimbConstants.motionMagicCruiseVelocity, ClimbConstants.motionMagicAcceleration,
        ClimbConstants.motionMagicJerk);

    // climbLeft
    // .getConfigurator()
    // .apply(climbConfig.withMotorOutput(new MotorOutputConfigs()
    // .withInverted(InvertedValue.CounterClockwise_Positive)
    // .withNeutralMode(NeutralModeValue.Brake)));
    // climbHook
    // .getConfigurator()
    // .apply(climbConfig.withMotorOutput(new MotorOutputConfigs()
    // .withInverted(InvertedValue.Clockwise_Positive)
    // .withNeutralMode(NeutralModeValue.Brake)));

    resetLeftPosition();
  }

  public void configMotionMagic(double cruiseVelocity, double acceleration, double jerk) {
    motionMagicConfigs.MotionMagicCruiseVelocity = cruiseVelocity;
    motionMagicConfigs.MotionMagicAcceleration = acceleration;
    motionMagicConfigs.MotionMagicJerk = jerk;

    climbLeft
        .getConfigurator()
        .apply(climbConfig.withMotorOutput(new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)));
  }

  public void setClimbLeft(Supplier<Double> position) {
    climbLeft.setControl(m_request.withPosition(position.get().doubleValue()));
  }

  public void setIsClimbing(boolean newIsClimbing) {
    isClimbing = newIsClimbing;
  }

  public boolean getIsClimbing() {
    return isClimbing;
  }

  public void resetLeftPosition() {
    // climbLeft.setControl(m_request.withPosition(0));
    climbLeft.setPosition(0);
  }

  public void setClimbLeftVoltage(double voltage) {
    climbLeft.setVoltage(voltage);
  }

  public boolean climbLeftAmpTriggered() {
    return Math.abs(climbLeft.getStatorCurrent().getValueAsDouble()) > ClimbConstants.climbCurrentLimit;
  }

  public double getLeftPosition() {
    return climbLeft.getPosition().getValueAsDouble();
  }

  public Command leftGoToPosition(Supplier<Double> position) {
    return new RunCommand(() -> setClimbLeft(position), this)
        .until(() -> Math.abs(getLeftPosition() - position.get()) <= ClimbConstants.positionToleranceRotations);
  }

  public Command leftResetPosition() {
    return new InstantCommand(() -> resetLeftPosition(), this);
  }

  public double getClimbLeftHeightInches() {
    return climbLeft.getPosition().getValueAsDouble()
        * ClimbConstants.inchesPerRotation(ClimbConstants.pitchDiameterInches);
  }

  @Override
  public void periodic() {
    // // This method will be called once per scheduler run
    // // Fake elevator motion using sine wave for simulation
    // double time = Timer.getFPGATimestamp();

    // // Height varies between 0.2m and 1.2m
    // double heightTall = 0.7 + 0.5 * Math.sin(time);
    // double heightShort = 0.5 + 0.1 * Math.sin(time);

    // tallMech.setLength(getclimbLeftHeightMeters());
    // shortMech.setLength(getclimbRightHeightMeters());

    SmartDashboard.putNumber("climb left position", climbLeft.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("climb left current (amps)", climbLeft.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("target position", ClimbConstants.l1Position);
  }
}