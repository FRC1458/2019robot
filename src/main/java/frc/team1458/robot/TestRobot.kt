package frc.team1458.robot

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.I2C
import edu.wpi.first.wpilibj.SampleRobot
import com.fazecast.jSerialComm.*
import edu.wpi.first.wpilibj.Timer
import frc.team1458.lib.actuator.SmartMotor
import frc.team1458.lib.drive.ClosedLoopTank
import frc.team1458.lib.pid.PIDConstants
import frc.team1458.lib.sensor.LineSensor
import frc.team1458.lib.sensor.SerialCommunication
import frc.team1458.lib.util.flow.delay
import java.io.FileWriter
import java.io.IOException

class TestRobot : SampleRobot() {

    override fun robotInit() {

        /*for (talon in talonArrayLeft) {
            talon.enableVoltageCompensation(true)
            talon.set(ControlMode.PercentOutput, 0.0)
        }

        for (talon in talonArrayRight) {
            talon.enableVoltageCompensation(true)
            talon.set(ControlMode.PercentOutput, 0.0)
        }*/
    }

    override fun autonomous() {}

    override fun operatorControl() {}

    override fun test() {
        System.setProperty("os.arch_full", "armv7")

        val comm = SerialCommunication(this)
        comm.startThread()

        while (true) {
            delay(1)
        }

        comm.endThread()


        /*
        var tmpVelocity = 0.0
        var updateCounter = 0
        var currentTime = 0.0
        var lastTime = 0.0
        var newRaw = 0.0
        var lastRaw = 0.0

        /*for (talon in talonArrayLeft) {
            talon.enableVoltageCompensation(true)
            talon.set(ControlMode.PercentOutput, 0.50)
        }

        for (talon in talonArrayRight) {
            talon.enableVoltageCompensation(true)
            talon.set(ControlMode.PercentOutput, -0.50)
        }*/

        dt.setDriveVelocity(6.0 * 1.0, 6.0 * 1.0)

        startTime = Timer.getFPGATimestamp() * 1000.0
        lastTime = startTime

        navx.zeroYaw()

        while (((Timer.getFPGATimestamp() * 1000.0) <= (startTime + runMs)) && isTest && isEnabled) {
            currentTime = Timer.getFPGATimestamp() * 1000.0

            newRaw = navx.rawAccelX.toDouble()
            tmpVelocity = newRaw * ((currentTime - lastTime) / 1000.0)

            if (newRaw != lastRaw) {
                velocity += tmpVelocity
                // println("m/s - $velocity")
                updateCounter += 1

                timestamp = ((currentTime) - this.startTime)
                velocityTimeData.add(arrayOf(timestamp, velocity, newRaw))

                lastRaw = newRaw
                lastTime = currentTime
            }


            /*
            print("Timestamp: $timestamp ")
            print("Velocity: $velocity ")
            */

            // println("HZ: ${1.0 / ((Timer.getFPGATimestamp()) - (tStart / 1000.0))}")
        }
        */

        /*for (talon in talonArrayLeft) {
            talon.enableVoltageCompensation(true)
            talon.set(ControlMode.PercentOutput, 0.0)
        }

        for (talon in talonArrayRight) {
            talon.enableVoltageCompensation(true)
            talon.set(ControlMode.PercentOutput, 0.0)
        }*/

        /*
        dt.setDriveVelocity(0.0, 0.0)

        var fileWriter: FileWriter? = null

        try {
            fileWriter = FileWriter("/tmp/test_data.csv")

            fileWriter.append("time,velocity")
            fileWriter.append('\n')

            for (point in velocityTimeData) {
                fileWriter.append(point[0].toString())
                fileWriter.append(',')
                fileWriter.append(point[1].toString())
                fileWriter.append(',')
                fileWriter.append(point[2].toString())
                fileWriter.append('\n')
            }

            println("Write CSV successfully!")
            println("Updated $updateCounter times in $runMs ms")
        } catch (e: Exception) {
            println("Writing CSV error!")
            e.printStackTrace()
        }
        try {
            fileWriter!!.flush()
            fileWriter.close()
        } catch (e: IOException) {
            println("Flushing/closing error!")
            e.printStackTrace()
        }
        */
    }

    override fun disabled() {}

}
