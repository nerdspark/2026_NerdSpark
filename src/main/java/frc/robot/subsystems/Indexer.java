package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.field;
import frc.robot.Constants.indexerConstants;
import frc.robot.Constants.turretConstants;
import frc.robot.commands.IndexerCommand;

public class Indexer implements Subsystem {

    private TalonFX spinDexerMotor;
    private Supplier<Double> rollerSpeed = () -> 0.0;
    

    public Indexer() {

        spinDexerMotor = new TalonFX(0); //TODO add these in Later
        // DrumMotor = new TalonFX(1); // TODO add these later
        
        // indexer configs
        TalonFXConfiguration spindexerConfig = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(40))
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

        // drum motor configs: 
        // TalonFXConfiguration DrumMotorConfig = new TalonFXConfiguration()
        //     .withCurrentLimits(new CurrentLimitsConfigs()
        //         .withStatorCurrentLimit(Amps.of(40))
        //         .withStatorCurrentLimitEnable(true)
        //     )
        // ;

        

        spinDexerMotor.getConfigurator().apply(spindexerConfig);

        //set drum motor configs
        // DrumMotor.getConfigurator().apply(DrumMotorConfig);
        
        
    }
    
    // Moves the the pass through motors and spins the conveyer belt.
    public void spinDex(Supplier<Boolean> isActive, Supplier<Double> rollerSpeed) {
        
        this.rollerSpeed = rollerSpeed;

        if(isActive.get()){
            spinDexerMotor.set(rollerSpeed.get());
        } else {
            spinDexerMotor.set(0.0);
        }
        
    }

    // control the drum motors
    // public void moveDrumMotors(Supplier<Boolean> isActive) {
    //     if(isActive.get()){
    //         DrumMotor.set(indexerConstants.DRUM_MOTOR_SPEED);
    //     } else {
    //         DrumMotor.set(0.0);
    //     }
    // }

    public Command incrementSpeed(Supplier<Boolean> isActive, Supplier<Double> increment) {
        
        Supplier<Double> newRollerSpeed = () -> rollerSpeed.get() + increment.get();

        return new IndexerCommand(this, isActive,newRollerSpeed);
    }

    @Override
    public void periodic() {
        
    }

    
}
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
