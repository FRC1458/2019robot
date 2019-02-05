package main.java.frc.team1458.robot

import java.io.*
object ThreadingSucks2 {
    @JvmStatic
    fun main(args:Array<String>) {
        var t:myThread
        for (i in 0..9)
        {
            try
            {
                t = myThread(i)
            }
            catch (e:IOException) {
                println("Log not found")
                break
            }
            t.start()
        }
    }
    internal class myThread @Throws(IOException::class)
    constructor(i:Int):Thread() {
        var out:PrintWriter
        init{
            out = PrintWriter(FileWriter("log" + i + ".csv"))
        }
        public override fun run() {
            out.println("New Thread @" + this.toString())
            out.close()
        }
    }
}