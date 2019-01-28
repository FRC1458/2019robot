package frc.team1458.lib.pathfinding

data class QuinticPlannerPose(
    val x: Double,
    val y: Double,
    val theta: Double = 0.0,
    val velocity: Double = 0.0,
    val acceleration: Double = 0.0
)