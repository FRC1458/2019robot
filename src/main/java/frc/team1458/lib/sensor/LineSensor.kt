package frc.team1458.lib.sensor

import com.fazecast.jSerialComm.*
import edu.wpi.first.wpilibj.SampleRobot
import frc.team1458.lib.util.flow.delay
import frc.team1458.robot.TestRobot

class LineSensor(robot: TestRobot, portDesc: String = "/dev/ttyACM1") {
    private val arduinoPort = SerialPort.getCommPort(portDesc)
    private var isRunning = false

    @Volatile
    var currentValue = 0

    @Volatile
    var lastValue = 0

    private val serialThread = Thread(Runnable {
        try {
            while (this.isRunning && robot.isEnabled) {
                while (arduinoPort.bytesAvailable() == 0) {
                    Thread.sleep(1)
                }

                val data = ByteArray(arduinoPort.bytesAvailable())
                setVal((data[0] + 128))
                setVal(5)
            }
        } catch (e: Exception) {
            e.printStackTrace()
        }

        arduinoPort.closePort()
        this.isRunning = false
    })

    fun startComms() {
        this.isRunning = true
        arduinoPort.openPort()

        this.serialThread.start()
    }

    private fun setVal(dat: Int) {
        this.lastValue = dat
    }

    fun getVal(): Int {
        return currentValue
    }

    fun endThread() {
        this.isRunning = false

        while (serialThread.isAlive) {
            delay(0.1)
        }
    }
}