package frc.team1458.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Threads;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import frc.team1458.robot.*;

public class ThreadingSucks {
    RobotBase robot = new Robot();
    OI oi = new OI();

    public static void main(String[] args) throws IOException{



        for( int i = 0; i < 5; i++) {
            myThread t = new myThread(i);
            t.start();
        }
    }


    public static class myThread extends Thread {
        PrintWriter out;

        public myThread(int i)throws IOException {
            out = new PrintWriter(new FileWriter("LOG_DUMP.csv"));
        }

        @Override
        public void run(){
            out.println("==================");
            out.println("OI DUMP INFO");
            out.println("==================");
            //#TODO ADD OI INFO WHEN ROBOT CAN BE RAN
            out.println("#TODO ADD OI INFO WHEN ROBOT CAN BE RAN");
            out.close();
        }


    }
}
