// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

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

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
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

public class Climb extends SubsystemBase {
  private TalonFX climbShort, climbTall, climbKicker;
  private TalonFXConfiguration climbConfig = new TalonFXConfiguration();
  private boolean ampTriggered, ampTriggerStarted = false;
  private final TalonFXSimState tallSim;
  private final TalonFXSimState shortSim;

  // Mechanism 2d
  private final Mechanism2d climbMech;
  // Roots for left and right elevators
  private final MechanismRoot2d tallRoot;
  private final MechanismRoot2d shortRoot;

  // Elevator ligaments
  private final MechanismLigament2d tallMech;
  private final MechanismLigament2d shortMech;
  // End of Mechanism 2d
  // private final StructPublisher<Pose3d> leftElevatorPoseHub;
  // private final StructPublisher<Pose3d> rightElevatorPoseHub;

  // Start of Climb Sequence
  // L1-------------------------
  // Taller Arm goes to 30 inches and hooks on first rung and Small Arm goes to
  // highest position
  // Tall Arm pulls down until small arm then small arm hooks on first rung and
  // Tall Arm lets go
  // L2-------------------------
  // Tall Arm goes to second rung position and hooks on second rung
  // Small Arm unhooks from first rung
  // Kicker Arm goes out and pushes robot back and allows clearance
  // Tall Arm pulls down until small arm then small arm hooks on second rung and
  // Tall Arm lets go
  // L3-------------------------
  // Tall Arm goes to third rung position and hooks on third rung
  // Small Arm unhooks from second rung
  // Kicker Arm goes out and pushes robot back and allow clearance
  // Tall Arm pulls down until small arm then small arm hooks on third rung and
  // Tall Arm lets go.
  // End Climb---------------------

  /** Creates a new Climb. */
  public Climb() {
    climbTall = new TalonFX(ClimbConstants.kLeftID, ClimbConstants.canBus);
    climbShort = new TalonFX(ClimbConstants.kRightID, ClimbConstants.canBus);
    climbKicker = new TalonFX(ClimbConstants.kKickerID, ClimbConstants.canBus);
    // climbHook = new TalonFX(ClimbConstants.kHookID, ClimbConstants.canBus);
    tallSim = climbTall.getSimState();
    shortSim = climbShort.getSimState();

    climbMech = new Mechanism2d(3.0, 3.0);
    // Roots at bottom of elevators
    tallRoot = climbMech.getRoot("LeftRoot", 1.0, 0.2);
    shortRoot = climbMech.getRoot("RightRoot", 2.0, 0.2);

    // Vertical elevator rails
    tallMech = new MechanismLigament2d(
        "LeftElevator",
        0.1, // initial length
        90, // 90° = vertical
        6,
        new Color8Bit(0, 150, 255));

    shortMech = new MechanismLigament2d(
        "RightElevator",
        0.,
        90,
        6,
        new Color8Bit(255, 150, 0));

    tallRoot.append(tallMech);
    shortRoot.append(shortMech);

    SmartDashboard.putData("ClimbMechanism", climbMech);

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
            .withInverted(InvertedValue.Clockwise_Positive) // Change Clockwise/CounterClockwise based on testing
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

  @Override
  public void simulationPeriodic() {
    // double dt = 0.02;

    // Read the applied motor voltage
    double tallVoltage = tallSim.getMotorVoltage();
    double shortVoltage = shortSim.getMotorVoltage();

    tallSim.addRotorPosition(tallVoltage);
    shortSim.addRotorPosition(shortVoltage);
  }

  // public void robotAngle(Supplier<Double> position) {
  // setClimbControl(position);
  // }

  public void setClimbTall(Supplier<Double> position) {
    climbTall.setControl(new PositionVoltage(position.get().doubleValue()));
    // climbHook.setControl(new PositionVoltage(position));

  }

  public void setClimbShort(Supplier<Double> position) {
    climbShort.setControl(new PositionVoltage(position.get().doubleValue()));
  }

  // Check if this function is needed during testing.
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
    return new RunCommand(() -> setClimbTall(position), this)
        .until(() -> Math.abs(getLeftPosition() - position.get()) <= ClimbConstants.positionToleranceRotations);
  }

  public Command tallResetPosition() {
    return new InstantCommand(() -> resetTallPosition(), this);
  }

  public Command shortResetPosition() {
    return new InstantCommand(() -> resetShortPosition(), this);
  }

  public double getClimbTallHeightMeters() {
    return climbTall.getPosition().getValueAsDouble() * ClimbConstants.metersPerRotation;
  }

  public double getClimbShortHeightMeters() {
    return climbShort.getPosition().getValueAsDouble() * ClimbConstants.metersPerRotation;
  }

  @Override
  public void periodic() {
    // // This method will be called once per scheduler run
    // // Fake elevator motion using sine wave for simulation
    // double time = Timer.getFPGATimestamp();

    // // Height varies between 0.2m and 1.2m
    // double heightTall = 0.7 + 0.5 * Math.sin(time);
    // double heightShort = 0.5 + 0.1 * Math.sin(time);

    tallMech.setLength(getClimbTallHeightMeters());
    shortMech.setLength(getClimbShortHeightMeters());

    SmartDashboard.putNumber("climb left position", climbTall.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("climb right position", climbShort.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("climb kicker position", climbKicker.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("climb left current (amps)", climbTall.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("climb right current (amps)", climbShort.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("climb kicker current (amps)", climbKicker.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("target position", ClimbConstants.l1Position);
  }
}
