package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.Turret;

public class ShootTime {
    private Turret turret;
    private Optional<Alliance> alliance;
    
    public ShootTime(Turret turr) {
        turret = turr;
        alliance = DriverStation.getAlliance();
    }

    public boolean canShoot() {
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        // We're teleop enabled, compute.
        double matchTime = DriverStation.getMatchTime();
        double shootTime = matchTime + turret.tof + 2;
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
        if (gameData.isEmpty()) {
            return true;
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                // If we have invalid game data, assume hub is active.
                return true;
            }
        }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (shootTime > 130) {
            // Transition shift, hub is active.
            return true;
        } else if (shootTime > 105) {
            // Shift 1
            return shift1Active;
        } else if (shootTime > 80) {
            // Shift 2
            return !shift1Active;
        } else if (shootTime > 55) {
            // Shift 3
            return shift1Active;
        } else if (shootTime > 30) {
            // Shift 4
            return !shift1Active;
        } else {
            // End game, hub always active.
            return true;
        }
    }
}
