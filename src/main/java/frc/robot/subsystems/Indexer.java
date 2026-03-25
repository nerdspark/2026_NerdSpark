package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.IndexConfig;

public class Indexer implements Subsystem {
    private final CANBus canivore;
    private final TalonFX passThroughMotor, spindexerMotor;

    private final TorqueCurrentFOC foc = new TorqueCurrentFOC(0);

    public Indexer() {
        canivore = new CANBus(Constants.CANbus);
        passThroughMotor = new TalonFX(IndexConfig.passThroughId, canivore);
        spindexerMotor = new TalonFX(IndexConfig.indexId, canivore);

        TalonFXConfiguration passThroughConfig = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(IndexConfig.statorCurretLimit)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(IndexConfig.supplyCurretLimit)
                .withSupplyCurrentLimitEnable(true))
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast));
        TalonFXConfiguration indexConfig = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(IndexConfig.statorCurretLimit)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(IndexConfig.supplyCurretLimit)
                .withSupplyCurrentLimitEnable(true))
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast));
        
        passThroughMotor.getConfigurator().apply(passThroughConfig);
        spindexerMotor.getConfigurator().apply(indexConfig);
    }

    public void spinDex(Supplier<Double> rollerSpeed) {
        foc.Output = IndexConfig.statorCurretLimit * rollerSpeed.get();
        passThroughMotor.setControl(foc);
        spindexerMotor.setControl(foc);
    }

    public void stopPassThrough() {
        foc.Output = 0;
        passThroughMotor.setControl(foc);
        spindexerMotor.setControl(foc);
    }
}
