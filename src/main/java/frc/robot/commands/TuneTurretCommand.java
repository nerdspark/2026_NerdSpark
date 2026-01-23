package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.hoodTuningConstants;
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

        SmartDashboard.setDefaultBoolean(hoodTuningConstants.enableKey, hoodTuningConstants.defaultEnable);
        SmartDashboard.setDefaultBoolean(hoodTuningConstants.zeroKey, hoodTuningConstants.defaultZero);
        SmartDashboard.setDefaultNumber(hoodTuningConstants.kPKey, hoodTuningConstants.defaultKP);
        SmartDashboard.setDefaultNumber(hoodTuningConstants.kIKey, hoodTuningConstants.defaultKI);
        SmartDashboard.setDefaultNumber(hoodTuningConstants.kDKey, hoodTuningConstants.defaultKD);
        SmartDashboard.setDefaultNumber(hoodTuningConstants.setpointDegKey, hoodTuningConstants.defaultSetpointDeg);
    }

    @Override
    public void execute() {
        boolean turretEnabled = SmartDashboard.getBoolean(turretTuningConstants.enableKey, turretTuningConstants.defaultEnable);
        boolean hoodEnabled = SmartDashboard.getBoolean(hoodTuningConstants.enableKey, hoodTuningConstants.defaultEnable);
        if (SmartDashboard.getBoolean(turretTuningConstants.zeroKey, turretTuningConstants.defaultZero)) {
            turret.zeroSpinSimPosition();
            SmartDashboard.putBoolean(turretTuningConstants.zeroKey, false);
        }
        if (SmartDashboard.getBoolean(hoodTuningConstants.zeroKey, hoodTuningConstants.defaultZero)) {
            turret.zeroHoodSimPosition();
            SmartDashboard.putBoolean(hoodTuningConstants.zeroKey, false);
        }
        SmartDashboard.putBoolean(turretTuningConstants.activeKey, turretEnabled);
        SmartDashboard.putBoolean(hoodTuningConstants.activeKey, hoodEnabled);

        if (!turretEnabled && !hoodEnabled) {
            turret.clearManualSpinOverride();
            turret.clearManualHoodOverride();
            return;
        }

        if (turretEnabled) {
            double kP = SmartDashboard.getNumber(turretTuningConstants.kPKey, turretTuningConstants.defaultKP);
            double kI = SmartDashboard.getNumber(turretTuningConstants.kIKey, turretTuningConstants.defaultKI);
            double kD = SmartDashboard.getNumber(turretTuningConstants.kDKey, turretTuningConstants.defaultKD);
            double setpointDeg = SmartDashboard.getNumber(turretTuningConstants.setpointDegKey, turretTuningConstants.defaultSetpointDeg);

            turret.setSpinPidGains(kP, kI, kD);
            turret.setManualSpinSetpointDegrees(setpointDeg);
        } else {
            turret.clearManualSpinOverride();
        }

        if (hoodEnabled) {
            double kP = SmartDashboard.getNumber(hoodTuningConstants.kPKey, hoodTuningConstants.defaultKP);
            double kI = SmartDashboard.getNumber(hoodTuningConstants.kIKey, hoodTuningConstants.defaultKI);
            double kD = SmartDashboard.getNumber(hoodTuningConstants.kDKey, hoodTuningConstants.defaultKD);
            double setpointDeg = SmartDashboard.getNumber(hoodTuningConstants.setpointDegKey, hoodTuningConstants.defaultSetpointDeg);

            turret.setHoodPidGains(kP, kI, kD);
            turret.setManualHoodSetpointDegrees(setpointDeg);
        } else {
            turret.clearManualHoodOverride();
        }
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean(turretTuningConstants.activeKey, false);
        SmartDashboard.putBoolean(hoodTuningConstants.activeKey, false);
        turret.clearManualSpinOverride();
        turret.clearManualHoodOverride();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
