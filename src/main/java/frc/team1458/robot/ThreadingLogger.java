package frc.team1458.robot;

import edu.wpi.first.wpilibj.RobotBase;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.*;

//public class ThreadingLogger {
//    RobotBase robot = new Robot();
//    OI oi = new OI();
//    Robot rob = new Robot();
//
//    public static void main(String[] args) throws IOException {
//        try {
//            RobotBase robot = new Robot();
//            OI oi = new OI();
//            Robot rob = new Robot();
//        } catch (NullPointerException e) {
//            System.out.print("ERROR: Code is not bootable from a robot, yet is ran by a standing computer.");
//            return;
//        }
//        RobotBase robot = new Robot();
//        OI oi = new OI();
//        Robot rob = new Robot();
//
//
//        for (int i = 0; i != -1; i++) {
//            myThread t = new myThread(i, rob, oi);
//            t.start();
//        }
//    }
//
//
//    public static class myThread extends Thread {
//        PrintWriter out;
//        OI oi;
//        Robot rob;
//
//        public myThread(int i, Robot rob, OI oi) throws IOException {
//            out = new PrintWriter(new FileWriter("LOG_DUMP.csv"));
//            this.rob = rob;
//            this.oi = oi;
//        }
//
//
//        public void run() {
//
//
//
//            //Yes i know this is ling but it really makes it readable code.
//
//            out.print("Get intake enabled, get drive train enabled, getDT, getelv1, getelv2, elevator enavled, elevator speed, gyro,intake1, intakeenabled, drivetraininverted,mag1, mag2, times, velocities, isautonomous, disabled, enabled, newdataavailable, newdataavailable, test, intakeout, leftstick, elevatordown, elevatorup, intakein, rightstick, slowdownbutton, steeraxis, throttleaxis");
//            out.println("");
//            out.print(rob.getIntakeEnabled()+",");
//            out.print(rob.getDrivetrainInverted()+",");
//            out.print(rob.getDt()+",");
//            out.print(rob.getElev1()+",");
//            out.print(rob.getElev2()+",");
//            out.print(rob.getElevatorEnabled()+",");
//            out.print(rob.getElevatorSpeed()+",");
//            out.print(rob.getGyro()+",");
//            out.print(rob.getIntake1()+",");
//            out.print(rob.getIntakeEnabled()+",");
//            out.print(rob.getDrivetrainInverted()+",");
//            out.print(rob.getMag1()+",");
//            out.print(rob.getMag2()+",");
//            out.print(rob.getTimes()+",");
//            out.print(rob.getVelocities()+",");
//
//
//
//            out.print(rob.isAutonomous()+",");
//            out.print(rob.isDisabled()+",");
//            out.print(rob.isEnabled()+",");
//            out.print(rob.isNewDataAvailable()+",");
//            out.print(rob.isNewDataAvailable()+",");
//            out.print(rob.isTest()+",");
//            out.print(oi.getIntakeOut()+",");
//            out.print(oi.getLeftStick()+",");
//            out.print(oi.getElevatorDown()+",");
//            out.print(oi.getElevatorUp()+",");
//            out.print(oi.getIntakeIn()+",");
//            out.print(oi.getRightStick()+",");
//            out.print(oi.getSlowDownButton()+",");
//            out.print(oi.getSteerAxis()+",");
//            out.print(oi.getThrottleAxis()+",");
//
//
//            out.close();
//        }
//
//
//    }
//}

public class ThreadingLogger extends Thread{
    PrintWriter logger;
    HashMap<String, Object> data;

    public ThreadingLogger(PrintWriter logger, HashMap<String, Object> data){
        this.logger = logger;
        this.data = data;
    }

    @Override
    public void run(){
        int count = 0;
        while(true) {
            synchronized (this) {
                try {
                    this.wait(200);
                } catch (InterruptedException e) {
                    logger.println("InterruptedException, " + e.getMessage());
                    logger.close();
                    return;
                }
                //data present
                logger.println("log, " + ++count);
                logger.println("Time, " + System.currentTimeMillis());
                for (String key : data.keySet()) {
                    logger.println(key + data.get(key).toString());
                }
            }
        }
    }
}