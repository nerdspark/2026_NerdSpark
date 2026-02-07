package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.PassThroughConfig;

public class Indexer implements Subsystem {

    private TalonFX passThroughMotor;
    
    public Indexer() {
        passThroughMotor = new TalonFX(PassThroughConfig.passThroughId);
        
        TalonFXConfiguration passThroughConfig = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(PassThroughConfig.passThroughStatorCurrentLimit)
                .withStatorCurrentLimitEnable(true)
            )
        ;

        passThroughMotor.getConfigurator().apply(passThroughConfig);
    }
    
    // Moves the the pass through motors and spins the conveyer belt.
    public void passThrough(Supplier<Double> rollerSpeed) {
        passThroughMotor.set(rollerSpeed.get()); 
    }

    public void stopPassThrough() {
        passThroughMotor.set(0);
    }
}