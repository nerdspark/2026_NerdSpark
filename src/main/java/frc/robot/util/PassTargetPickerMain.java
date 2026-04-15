package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableInstance;

public class PassTargetPickerMain {
    private static final int DEFAULT_TEAM = 9312;

    public static void main(String[] args) throws InterruptedException {
        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        nt.startClient4("PassTargetPicker");

        if (args.length >= 1 && args[0].equalsIgnoreCase("sim")) {
            System.out.println("Connecting to localhost (sim mode)");
            nt.setServer("localhost");
        } else {
            int team = DEFAULT_TEAM;
            if (args.length >= 1) {
                try {
                    team = Integer.parseInt(args[0]);
                } catch (NumberFormatException e) {
                    System.err.println("Invalid team number: " + args[0] + ", using default " + DEFAULT_TEAM);
                }
            }
            System.out.println("Connecting to team " + team);
            nt.setServerTeam(team);
        }

        new PassTargetPicker().start();

        while (true) {
            Thread.sleep(1000);
        }
    }
}