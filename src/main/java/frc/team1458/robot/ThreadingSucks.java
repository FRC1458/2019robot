package frc.team1458.robot;

import edu.wpi.first.wpilibj.RobotBase;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;

public class ThreadingSucks {
    RobotBase robot = new Robot();
    OI oi = new OI();
    Robot rob = new Robot();

    public static void main(String[] args) throws IOException {
        try {
            RobotBase robot = new Robot();
            OI oi = new OI();
            Robot rob = new Robot();
        } catch (NullPointerException e) {
            System.out.print("ERROR: Code is not bootable from a robot, yet is ran by a standing computer.");
            return;
        }
        RobotBase robot = new Robot();
        OI oi = new OI();
        Robot rob = new Robot();


        for (int i = 0; i != -1; i++) {
            myThread t = new myThread(i, rob, oi);
            t.start();
        }
    }


    public static class myThread extends Thread {
        PrintWriter out;
        OI oi;
        Robot rob;

        public myThread(int i, Robot rob, OI oi) throws IOException {
            out = new PrintWriter(new FileWriter("LOG_DUMP.csv"));
            this.rob = rob;
            this.oi = oi;
        }


        public void run() {

            ArrayList everything = new ArrayList();

            //Yes i know this is ling but it really makes it readable code.
            everything.add(rob.getIntakeEnabled());
            everything.add(rob.getDrivetrainInverted());
            everything.add(rob.getDt());
            everything.add(rob.getElev1());
            everything.add(rob.getElev2());
            everything.add(rob.getElevatorEnabled());
            everything.add(rob.getElevatorSpeed());
            everything.add(rob.getGyro());
            everything.add(rob.getIntake1());
            everything.add(rob.getIntakeEnabled());
            everything.add(rob.getDrivetrainInverted());
            everything.add(rob.getMag1());
            everything.add(rob.getMag2());
            everything.add(rob.getTimes());
            everything.add(rob.getVelocities());

            everything.add(rob.isAutonomous());
            everything.add(rob.isDisabled());
            everything.add(rob.isEnabled());
            everything.add(rob.isNewDataAvailable());
            everything.add(rob.isNewDataAvailable());
            everything.add(rob.isTest());
            everything.add(oi.getIntakeOut());
            everything.add(oi.getLeftStick());
            everything.add(oi.getElevatorDown());
            everything.add(oi.getElevatorUp());
            everything.add(oi.getIntakeIn());
            everything.add(oi.getRightStick());
            everything.add(oi.getSlowDownButton());
            everything.add(oi.getSteerAxis());
            everything.add(oi.getThrottleAxis());

            //Log everything as one list
            out.print(everything);


            out.close();
        }


    }
}
