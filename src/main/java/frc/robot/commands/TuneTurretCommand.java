package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.turretTuningConstants;
import frc.robot.subsystems.Turret;

public class TuneTurretCommand extends Command {
    private final Turret turret;

    public TuneTurretCommand(Turret turret) {
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        SmartDashboard.setDefaultBoolean(turretTuningConstants.enableKey, turretTuningConstants.defaultEnable);
        SmartDashboard.setDefaultBoolean(turretTuningConstants.zeroKey, turretTuningConstants.defaultZero);
        SmartDashboard.setDefaultNumber(turretTuningConstants.kPKey, turretTuningConstants.defaultKP);
        SmartDashboard.setDefaultNumber(turretTuningConstants.kIKey, turretTuningConstants.defaultKI);
        SmartDashboard.setDefaultNumber(turretTuningConstants.kDKey, turretTuningConstants.defaultKD);
        SmartDashboard.setDefaultNumber(turretTuningConstants.setpointDegKey, turretTuningConstants.defaultSetpointDeg);
    }

    @Override
    public void execute() {
        boolean enabled = SmartDashboard.getBoolean(turretTuningConstants.enableKey, turretTuningConstants.defaultEnable);
        if (SmartDashboard.getBoolean(turretTuningConstants.zeroKey, turretTuningConstants.defaultZero)) {
            turret.zeroSimPosition();
            SmartDashboard.putBoolean(turretTuningConstants.zeroKey, false);
        }
        SmartDashboard.putBoolean(turretTuningConstants.activeKey, enabled);
        if (!enabled) {
            turret.clearManualOverride();
            return;
        }

        double kP = SmartDashboard.getNumber(turretTuningConstants.kPKey, turretTuningConstants.defaultKP);
        double kI = SmartDashboard.getNumber(turretTuningConstants.kIKey, turretTuningConstants.defaultKI);
        double kD = SmartDashboard.getNumber(turretTuningConstants.kDKey, turretTuningConstants.defaultKD);
        double setpointDeg = SmartDashboard.getNumber(turretTuningConstants.setpointDegKey, turretTuningConstants.defaultSetpointDeg);

        turret.setSpinPidGains(kP, kI, kD);
        turret.setManualSetpointDegrees(setpointDeg);
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean(turretTuningConstants.activeKey, false);
        turret.clearManualOverride();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
