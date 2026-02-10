// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

public class ShortClimb extends SubsystemBase {
  private TalonFX climbShort;
  private TalonFXConfiguration climbConfig = new TalonFXConfiguration();
  private boolean ampTriggered, ampTriggerStarted = false;
  // private final TalonFXSimState tallSim;
  // private final TalonFXSimState shortSim;

  private MotionMagicConfigs motionMagicConfigs = climbConfig.MotionMagic;
  final MotionMagicVoltage m_request = new MotionMagicVoltage(0);


  // Mechanism 2d
  // private final Mechanism2d climbMech;
  // Roots for left and right elevators

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
  public ShortClimb() {
    climbShort = new TalonFX(ClimbConstants.kRightID, ClimbConstants.canBus);
    // climbHook = new TalonFX(ClimbConstants.kHookID, ClimbConstants.canBus);
    // shortSim = climbShort.getSimState();

    // climbMech = new Mechanism2d(3.0, 3.0);
    // // Roots at bottom of elevators

    // SmartDashboard.putData("ClimbMechanism", climbMech);

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

    climbShort
        .getConfigurator()
        .apply(climbConfig.withMotorOutput(new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)));

    // climbHook
    // .getConfigurator()
    // .apply(climbConfig.withMotorOutput(new MotorOutputConfigs()
    // .withInverted(InvertedValue.Clockwise_Positive)
    // .withNeutralMode(NeutralModeValue.Brake)));

    resetShortPosition();
  }

  @Override
  public void simulationPeriodic() {
    // // double dt = 0.02;

    // // Read the applied motor voltage
    // double tallVoltage = tallSim.getMotorVoltage();
    // double shortVoltage = shortSim.getMotorVoltage();

    // tallSim.addRotorPosition(tallVoltage);
    // shortSim.addRotorPosition(shortVoltage);
  }

  // public void robotAngle(Supplier<Double> position) {
  // setClimbControl(position);
  // }

  public void setClimbShort(Supplier<Double> position) {
    climbShort.setControl(m_request.withPosition(position.get().doubleValue()));
  } 

  public void resetShortPosition() {
    climbShort.setPosition(0);
  }

  public void setClimbShortVoltage(double voltage) {
    climbShort.setVoltage(voltage);
  }

  public boolean climbRightAmpTriggered() {
    return Math.abs(climbShort.getStatorCurrent().getValueAsDouble()) > ClimbConstants.climbCurrentLimit;
  }

  public double getRightPosition() {
    return climbShort.getPosition().getValueAsDouble();
  }

  public Command shortGoToPosition(Supplier<Double> position) {
    return new RunCommand(() -> setClimbShort(position), this)
        .until(() -> Math.abs(getRightPosition() - position.get()) <= ClimbConstants.positionToleranceRotations);
  }


  public Command shortResetPosition() {
    return new InstantCommand(() -> resetShortPosition(), this);
  }

  public double getClimbShortHeightMeters() {
    return climbShort.getPosition().getValueAsDouble() * ClimbConstants.metersPerRotation(ClimbConstants.pitchDiameterMeters);
  }

  @Override
  public void periodic() {
    // // This method will be called once per scheduler run
    // // Fake elevator motion using sine wave for simulation
    // double time = Timer.getFPGATimestamp();

    // // Height varies between 0.2m and 1.2m
    // double heightTall = 0.7 + 0.5 * Math.sin(time);
    // double heightShort = 0.5 + 0.1 * Math.sin(time);

    // shortMech.setLength(getClimbShortHeightMeters());

    SmartDashboard.putNumber("climb right position", climbShort.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("climb right current (amps)", climbShort.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("target position", ClimbConstants.l1Position);
  }
}
