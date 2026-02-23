package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.IndexConfig;

public class Indexer implements Subsystem {
    private CANBus canivore;
    private TalonFX passThroughMotor, indexMotor;
    
    public Indexer() {
        canivore = new CANBus(Constants.CANbus);
        passThroughMotor = new TalonFX(IndexConfig.passThroughId, canivore);
        indexMotor = new TalonFX(IndexConfig.indexId, canivore);
        
        TalonFXConfiguration passThroughConfig = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(IndexConfig.passThroughStatorCurrentLimit)
                .withStatorCurrentLimitEnable(true)
            )
        ;
        TalonFXConfiguration indexConfig = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(IndexConfig.indexCurretLimit)
                .withStatorCurrentLimitEnable(true)
            )
        ;

        passThroughMotor.getConfigurator().apply(passThroughConfig);
        indexMotor.getConfigurator().apply(indexConfig);
    }
    
    // Moves the the pass through motors and spins the conveyer belt.
    public void passThrough(Supplier<Double> rollerSpeed) {
        passThroughMotor.set(rollerSpeed.get());
        indexMotor.set(rollerSpeed.get()); 
    }

    public void stopPassThrough() {
        passThroughMotor.set(0);
        indexMotor.set(0);
    }
}