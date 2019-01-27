package frc.team1458.robot

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.I2C
import edu.wpi.first.wpilibj.SampleRobot
import edu.wpi.first.wpilibj.Timer
import frc.team1458.lib.actuator.SmartMotor
import frc.team1458.lib.drive.ClosedLoopTank
import frc.team1458.lib.pid.PIDConstants
import java.io.FileWriter
import java.io.IOException

class TestRobot : SampleRobot() {

    val dt: ClosedLoopTank = ClosedLoopTank(
        leftMaster = SmartMotor.CANtalonSRX(16),
        rightMaster = SmartMotor.CANtalonSRX(22).inverted,
        leftMotors = arrayOf(SmartMotor.CANtalonSRX(25)),
        rightMotors = arrayOf(SmartMotor.CANtalonSRX(23).inverted),
        closedLoopControl = false,
        wheelDiameter = 0.5,
        closedLoopScaling = 6.0, // TODO: determine

        // TODO Don't forget PID I is disabled for autonomous testing as it introduces errors: maybe 0.0 for I
        pidConstantsLeft = PIDConstants(0.6, kI = 0.001/*3*/, kD = 0.00, kF = 1.0 / 1798.8), // These are decent
        pidConstantsRight = PIDConstants(0.750, kI = 0.001/*5*/, kD = 0.05, kF = 1.0 / 1806.4)// These are decent2
    )

    /* Talon Controllers
    val leftOne: TalonSRX = TalonSRX(16)
    val leftTwo: TalonSRX = TalonSRX(25)

    val rightOne: TalonSRX = TalonSRX(22)
    val rightTwo: TalonSRX = TalonSRX(23)

    val talonArrayLeft = arrayOf(leftOne, leftTwo)
    val talonArrayRight = arrayOf(rightOne, rightTwo)
    */

    // Testing Stuff
    val navx: AHRS = AHRS(I2C.Port.kMXP)

    val velocityTimeData: ArrayList<Array<Double>> = ArrayList<Array<Double>>()
    var timestamp: Double = 0.0
    var velocity: Double = 0.0
    var startTime: Double = Timer.getFPGATimestamp() * 1000.0
    val runMs: Double = 5000.0


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

        /*for (talon in talonArrayLeft) {
            talon.enableVoltageCompensation(true)
            talon.set(ControlMode.PercentOutput, 0.0)
        }

        for (talon in talonArrayRight) {
            talon.enableVoltageCompensation(true)
            talon.set(ControlMode.PercentOutput, 0.0)
        }*/

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
    }

    override fun disabled() {}

}
