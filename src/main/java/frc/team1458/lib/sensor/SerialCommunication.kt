package frc.team1458.lib.sensor

import com.fazecast.jSerialComm.*
import frc.team1458.lib.core.BaseRobot
import frc.team1458.lib.util.flow.delay
import frc.team1458.robot.TestRobot
import kotlin.math.abs

class SerialCommunication(robot: TestRobot, portDesc: String = "/dev/ttyACM0") {
    private val serialPort = SerialPort.getCommPort(portDesc) // SerialPort.getCommPort(portDesc)
    private var isRunning = false

    private val serialThread = Thread(Runnable {
        var byteBuffer = ByteArray(serialPort.bytesAvailable())

        try {
            while (this.isRunning && robot.isEnabled) {
                while (serialPort.bytesAvailable() == 0) {}
                byteBuffer = ByteArray(serialPort.bytesAvailable())
                serialPort.readBytes(byteBuffer, byteBuffer.size.toLong())
                val startByte = byteBuffer[0].toInt()

                if (startByte == 0x7f) {

                    while (serialPort.bytesAvailable() == 0) {}
                    byteBuffer = ByteArray(serialPort.bytesAvailable())
                    serialPort.readBytes(byteBuffer, byteBuffer.size.toLong())
                    val firstDataByte = byteBuffer[0].toInt() + 128 // TODO check
                    System.out.println("First Byte: $firstDataByte")
                    if (firstDataByte == 255) {
                        continue
                    }

                    while (serialPort.bytesAvailable() == 0) {}
                    byteBuffer = ByteArray(serialPort.bytesAvailable())
                    serialPort.readBytes(byteBuffer, byteBuffer.size.toLong())
                    val secondDataByte = byteBuffer[0].toInt() + 128 // TODO check

                    if (secondDataByte == 255) {
                        continue
                    }
                    System.out.println("Second Byte: $secondDataByte")

                    while (serialPort.bytesAvailable() == 0) {}
                    byteBuffer = ByteArray(serialPort.bytesAvailable())
                    serialPort.readBytes(byteBuffer, byteBuffer.size.toLong())
                    val checkSum = byteBuffer[0].toInt()
                    System.out.println("Checksum Byte: $checkSum vs ${(abs(firstDataByte + secondDataByte) % 256)}")

                    if (checkSum == 255) {
                        continue
                    }

                    if ((abs(firstDataByte + secondDataByte) % 256) == checkSum) {
                        System.out.println("CheckSum Verified!")
                        while (serialPort.bytesAvailable() == 0) {}
                        byteBuffer = ByteArray(serialPort.bytesAvailable())
                        serialPort.readBytes(byteBuffer, byteBuffer.size.toLong())
                        val endByte = byteBuffer[0].toInt()

                        if (endByte == 0xff) {
                            System.out.println("Data Complete! - $firstDataByte & $secondDataByte")
                        }
                    }
                }

            }
        } catch (e: Exception) {
            e.printStackTrace()
        }

        serialPort.closePort()
        this.isRunning = false
    })

    fun startThread() {
        serialPort.setComPortTimeouts(SerialPort.TIMEOUT_NONBLOCKING, 0, 0)
        serialPort.openPort()
        this.isRunning = true
        serialThread.start()
    }

    fun endThread() {
        this.isRunning = false
        while (serialThread.isAlive) {
            delay(1)
        }
    }
}