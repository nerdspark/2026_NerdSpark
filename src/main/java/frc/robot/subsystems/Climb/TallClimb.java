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

public class TallClimb extends SubsystemBase {
 private TalonFX climbTall, climbShort;
  private TalonFXConfiguration climbConfig = new TalonFXConfiguration();
  private boolean ampTriggered, ampTriggerStarted = false;
  // private final TalonFXSimState tallSim;
  // private final TalonFXSimState shortSim;

  private MotionMagicConfigs motionMagicConfigs = climbConfig.MotionMagic;
  final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

  /** Creates a new TallClimb. */
  public TallClimb() {
    climbTall = new TalonFX(ClimbConstants.kLeftID, ClimbConstants.canBus);

    // Initializing the motor
    climbConfig.CurrentLimits = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(ClimbConstants.climbCurrentLimit)
        .withStatorCurrentLimitEnable(true);
    climbConfig.Feedback = new FeedbackConfigs()
        .withFeedbackRotorOffset(0)
        .withSensorToMechanismRatio(ClimbConstants.sensorToMechanismRatio);
    // climbConfig.ClosedLoopRamps = new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(ClimbConstants.rampRate);
    climbConfig.Slot0 = new Slot0Configs()
        .withKP(ClimbConstants.kP)
        .withKI(ClimbConstants.kI)
        .withKD(ClimbConstants.kD)
        .withKG(ClimbConstants.kG)
        .withKS(ClimbConstants.kS)
        .withKA(ClimbConstants.kA)
        .withKV(ClimbConstants.kV)
        .withGravityType(GravityTypeValue.Elevator_Static);

    motionMagicConfigs.MotionMagicCruiseVelocity = ClimbConstants.motionMagicCruiseVelocity;
    motionMagicConfigs.MotionMagicAcceleration = ClimbConstants.motionMagicAcceleration;
    motionMagicConfigs.MotionMagicJerk = ClimbConstants.motionMagicJerk;

    climbTall
        .getConfigurator()
        .apply(climbConfig.withMotorOutput(new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)));
    // climbHook
    // .getConfigurator()
    // .apply(climbConfig.withMotorOutput(new MotorOutputConfigs()
    // .withInverted(InvertedValue.Clockwise_Positive)
    // .withNeutralMode(NeutralModeValue.Brake)));

    resetTallPosition();
  }

  public void setClimbTall(Supplier<Double> position) {
    climbTall.setControl(m_request.withPosition(position.get().doubleValue()));
  }



  public void resetTallPosition() {
    // climbTall.setControl(m_request.withPosition(0));
    climbTall.setPosition(0);
  }

  public void setClimbTallVoltage(double voltage) {
    climbTall.setVoltage(voltage);
  }

  public boolean climbLeftAmpTriggered() {
    return Math.abs(climbTall.getStatorCurrent().getValueAsDouble()) > ClimbConstants.climbCurrentLimit;
  }


  public double getLeftPosition() {
    return climbTall.getPosition().getValueAsDouble();
  }

  public Command tallGoToPosition(Supplier<Double> position) {
    return new RunCommand(() -> setClimbTall(position), this)
        .until(() -> Math.abs(getLeftPosition() - position.get()) <= ClimbConstants.positionToleranceRotations);
  }

  public Command tallResetPosition() {
    return new InstantCommand(() -> resetTallPosition(), this);
  }


  public double getClimbTallHeightInches() {
    return climbTall.getPosition().getValueAsDouble() * ClimbConstants.inchesPerRotation(ClimbConstants.pitchDiameterInches);
  }

  @Override
  public void periodic() {
    // // This method will be called once per scheduler run
    // // Fake elevator motion using sine wave for simulation
    // double time = Timer.getFPGATimestamp();

    // // Height varies between 0.2m and 1.2m
    // double heightTall = 0.7 + 0.5 * Math.sin(time);
    // double heightShort = 0.5 + 0.1 * Math.sin(time);

    // tallMech.setLength(getClimbTallHeightMeters());
    // shortMech.setLength(getClimbShortHeightMeters());

    SmartDashboard.putNumber("climb left position", climbTall.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("climb left current (amps)", climbTall.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("target position", ClimbConstants.l1Position);
  }
}
