package frc.team1458.robot

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.I2C
import edu.wpi.first.wpilibj.SampleRobot
import edu.wpi.first.wpilibj.Timer
import java.io.FileWriter
import java.io.IOException

class TestRobot : SampleRobot() {

    // val oi: OI = OI()

    // Talon Controllers
    val leftOne: TalonSRX = TalonSRX(16)
    val leftTwo: TalonSRX = TalonSRX(25)

    val rightOne: TalonSRX = TalonSRX(22)
    val rightTwo: TalonSRX = TalonSRX(23)

    val talonArray = arrayOf(leftOne, leftTwo, rightOne, rightTwo)

    // Testing Stuff
    val navx: AHRS = AHRS(I2C.Port.kMXP)

    val velocityTimeData: ArrayList<Pair<Double, Double>> = ArrayList<Pair<Double, Double>>()
    var timestamp: Double = 0.0
    var velocity: Double = 0.0
    val startTime: Double = Timer.getFPGATimestamp() * 1000.0


    override fun robotInit() {

        for (talon in talonArray) {
            talon.enableVoltageCompensation(true)
            talon.set(ControlMode.PercentOutput, 0.0)
        }
    }

    override fun autonomous() {}

    override fun operatorControl() {}

    override fun test() {
        var tStart: Double

        for (talon in this.talonArray) {
            talon.set(ControlMode.PercentOutput, 0.25)
        }

        while ((System.currentTimeMillis().toDouble() / 1000.0) <= ((startTime / 1e-9) + 2.0)) {
            tStart = Timer.getFPGATimestamp() * 1000.0

            timestamp = (((Timer.getFPGATimestamp() * 1000.0) - this.startTime))
            velocity = navx.velocityX.toDouble()

            velocityTimeData.add(Pair(timestamp, velocity))

            /*
            print("Timestamp: $timestamp ")
            print("Velocity: $velocity ")
            */

            println("HZ: ${1.0 / ((Timer.getFPGATimestamp()) - (tStart / 1000.0))}")
        }

        for (talon in this.talonArray) {
            talon.set(ControlMode.PercentOutput, 0.0)
        }

        var fileWriter: FileWriter? = null

        try {
            fileWriter = FileWriter("/tmp/test_data.csv")

            fileWriter.append("time,velocity")
            fileWriter.append('\n')

            for (point in velocityTimeData) {
                fileWriter.append(point.first.toString())
                fileWriter.append(',')
                fileWriter.append(point.second.toString())
                fileWriter.append('\n')
            }

            println("Write CSV successfully!")
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
    }

    override fun disabled() {}

}
