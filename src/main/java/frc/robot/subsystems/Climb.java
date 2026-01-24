// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {
    private TalonFX climbRight, climbLeft, climbHook;
    private TalonFXConfiguration climbConfig = new TalonFXConfiguration();
    private boolean ampTriggered, ampTriggerStarted = false;
  /** Creates a new Climb. */
  public Climb() {
    climbLeft = new TalonFX(ClimbConstants.kLeftID, ClimbConstants.canBus);
    climbRight = new TalonFX(ClimbConstants.kRightID, ClimbConstants.canBus);
    climbHook = new TalonFX(ClimbConstants.kHookID, ClimbConstants.canBus);
//Initializing the motor
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
      .withKD(ClimbConstants.kD);
      climbLeft
          .getConfigurator()
          .apply(climbConfig.withMotorOutput(new MotorOutputConfigs()
          .withInverted(InvertedValue.CounterClockwise_Positive)
              .withNeutralMode(NeutralModeValue.Brake)));
      climbRight
          .getConfigurator()
          .apply(climbConfig.withMotorOutput(new MotorOutputConfigs()
          .withInverted(InvertedValue.Clockwise_Positive)
              .withNeutralMode(NeutralModeValue.Brake)));
      climbHook
        .getConfigurator()
            .apply(climbConfig.withMotorOutput(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake)));
      
      resetPosition();
  }
  public void robotAngle(double position) {
    setPosition(position);
  }

  public void setPosition (double position){
    climbLeft.setControl(new PositionVoltage(position));
    climbRight.setControl(new PositionVoltage(position));
    climbHook.setControl(new PositionVoltage(position));

  }

  public void getHookPosition(){
    
  }
  public void resetPosition(){

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}

//skibidi2