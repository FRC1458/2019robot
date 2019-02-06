package frc.team1458.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Threads;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

import frc.team1458.lib.input.FlightStick;
import frc.team1458.robot.*;

public class ThreadingSucks {
    RobotBase robot = new Robot();
    OI oi = new OI();
    Robot rob = new Robot();
    public static void main(String[] args) throws IOException{
        try {
            RobotBase robot = new Robot();
            OI oi = new OI();
            Robot rob = new Robot();
        }
        catch(NullPointerException e) {
            System.out.println("ERROR: Code is not bootable from a robot, yet is ran by a standing computer.");
            return;
        }
        RobotBase robot = new Robot();
        OI oi = new OI();
        Robot rob = new Robot();



        for( int i = 0; i < 5; i++) {
            myThread t = new myThread(i,rob,oi);
            t.start();
        }
    }


    public static class myThread extends Thread {
        PrintWriter out;
        OI oi;
        Robot rob;

        public myThread(int i, Robot rob, OI oi)throws IOException {
            out = new PrintWriter(new FileWriter("LOG_DUMP.csv"));
            this.rob = rob;
            this.oi = oi;
        }


        public void run(){
            out.println("==================");
            out.println("ROBOT DUMP INFO");
            out.println("==================");
            //#TODO ADD OI INFO WHEN ROBOT CAN BE RAN
            out.println("Intake enabled : "+rob.getIntakeEnabled());
            out.println("Drive train inverted : "+rob.getDrivetrainInverted());
            out.println("DT : "+rob.getDt());
            out.println("Elv1 : "+rob.getElev1());
            out.println("Elv2 : "+rob.getElev2());
            out.println("Elevator enabled : "+rob.getElevatorEnabled());
            out.println("Elevator speed : "+rob.getElevatorSpeed());
            out.println("Gyro : "+rob.getGyro());
            out.println("Intake 1 : "+rob.getIntake1());
            out.println("Intake enabled : "+rob.getIntakeEnabled());
            out.println("DriveTrain Inverted : "+rob.getDrivetrainInverted());
            out.println("Mag1 : "+rob.getMag1());
            out.println("Mag2 : "+rob.getMag2());
            out.println("Times : "+rob.getTimes());
            out.println("Velocities : "+rob.getVelocities());

            out.println("---------");
            out.println("isAutonomous : "+rob.isAutonomous());
            out.println("isDisabled : "+rob.isDisabled());
            out.println("isEnabled : "+rob.isEnabled());
            out.println("isNewDataAvailable : "+rob.isNewDataAvailable());
            out.println("isNewDataAvailable : "+rob.isNewDataAvailable());
            out.println("isTest : "+rob.isTest());

            out.println("==================");
            out.println("OI DUMP INFO");
            out.println("==================");


            out.println("Intake out : "+oi.getIntakeOut());
            out.println("Left Stick : "+oi.getLeftStick());
            out.println("Elevator Down : "+oi.getElevatorDown());
            out.println("Elevator Up : "+oi.getElevatorUp());
            out.println("Intake in  : "+oi.getIntakeIn());
            out.println("Right Stick : "+oi.getRightStick());
            out.println("Slow Down Button : "+oi.getSlowDownButton());
            out.println("Steer axis : "+oi.getSteerAxis());
            out.println("Throttle axis : "+oi.getThrottleAxis());






            out.close();
        }


    }
}
