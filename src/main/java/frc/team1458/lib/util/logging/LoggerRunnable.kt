package frc.team1458.lib.util.logging

import java.io.File
import java.io.BufferedWriter
import java.io.FileWriter

import frc.team1458.lib.util.flow.systemTimeMillis

class LoggerRunnable(private val filePath: String,
                     private val keys: Array<String>,
                     private val intervalMS: Int = 15) : Runnable  {

    @Volatile private var inputArray: Array<String?> = arrayOfNulls(keys.size)

    private var fileWriter: FileWriter? = null
    private var bufferedWriter: BufferedWriter? = null

    @Volatile var logging: Boolean = true

    @Synchronized fun updateKey(key: String, value: Any) {
        inputArray[keys.indexOf(key)] = value.toString() // TODO Make sure "Any" works well
    }

    fun setup() {
        val file = File(filePath)

        if (!file.exists()) {
            file.createNewFile()
        }

        fileWriter = FileWriter(file.absoluteFile)
        bufferedWriter = BufferedWriter(fileWriter)

        bufferedWriter?.write("ts")

        for (key in keys) {
            bufferedWriter?.write(",$key")
        }

        bufferedWriter?.newLine()
    }

    fun stop() {
        logging = false
    }

    override fun run() {
        while (logging) { // TODO Make it not go on forever...
            bufferedWriter?.write("$systemTimeMillis")

            for (input in inputArray) {
                bufferedWriter?.write(",${input.toString()}")
            }

            bufferedWriter?.newLine()

            Thread.sleep(intervalMS.toLong())
        }

        bufferedWriter?.close()
    }
}