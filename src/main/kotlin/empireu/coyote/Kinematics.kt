package empireu.coyote

data class MecanumVelocitiesDual(
    val frontLeft: Dual,
    val frontRight: Dual,
    val backLeft: Dual,
    val backRight: Dual
) {
    override fun toString() = "FL=$frontLeft FR=$frontRight BL=$backLeft BR=$backRight"
}

/**
 * Performs **Inverse Mecanum Drive Kinematics** to transform [velRobot] to *linear wheel velocities*, using equations derived from [Inverse kinematic implementation of four-wheels mecanum drive mobile robot using stepper motors](https://www.researchgate.net/publication/308570348_Inverse_kinematic_implementation_of_four-wheels_mecanum_drive_mobile_robot_using_stepper_motors)
 * */
fun mecanumInvKinematics(velRobot: Twist2dDual, a: Double, b: Double): MecanumVelocitiesDual {
    val vx = velRobot.trVelocity.x
    val vy = velRobot.trVelocity.y
    val w = velRobot.rotVelocity
    val aw = a * w
    val bw = b * w

    return MecanumVelocitiesDual(
        vx - vy - aw - bw,
        vx + vy + aw + bw,
        vx + vy - aw - bw,
        vx - vy + aw + bw,
    )
}