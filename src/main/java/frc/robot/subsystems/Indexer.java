package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.IndexConfig;

public class Indexer implements Subsystem {
    private final CANBus canivore;
    private final TalonFX passThroughMotor;
    private final TalonFX spindexerMotor;

    public Indexer() {

        canivore = new CANBus(Constants.CANbus);
        passThroughMotor = new TalonFX(IndexConfig.passThroughId, canivore);
        spindexerMotor = new TalonFX(IndexConfig.indexId, canivore);

        TalonFXConfiguration passThroughConfig = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(IndexConfig.passThroughStatorCurrentLimit)
                .withStatorCurrentLimitEnable(true));
        TalonFXConfiguration indexConfig = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(IndexConfig.indexCurretLimit)
                .withStatorCurrentLimitEnable(true))
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast));
        
        passThroughMotor.getConfigurator().apply(passThroughConfig);
        spindexerMotor.getConfigurator().apply(indexConfig);
    }

    public void spinDex( Supplier<Double> rollerSpeed) {
        double speed = rollerSpeed.get();
        passThroughMotor.set(speed);
        spindexerMotor.set(speed);
    }

    public void stopPassThrough() {
        passThroughMotor.set(0.0);
        spindexerMotor.set(0.0);
    }

    public edu.wpi.first.wpilibj2.command.Command incrementSpeed(
        Supplier<Boolean> isActive,
        Supplier<Double> increment
    ) {
        return edu.wpi.first.wpilibj2.command.Commands.none();
    }
}
