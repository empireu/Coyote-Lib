package empireu.coyote

object MecanumKinematics {
    data class MecanumVelocitiesDual(val frontLeft: Dual, val frontRight: Dual, val backLeft: Dual, val backRight: Dual) {
        override fun toString() = "FL=$frontLeft FR=$frontRight BL=$backLeft BR=$backRight"
    }

    /**
     * Performs **Inverse Mecanum Drive Kinematics** to transform [velRobot] to *linear wheel velocities*, using equations derived from [Inverse kinematic implementation of four-wheels mecanum drive mobile robot using stepper motors](https://www.researchgate.net/publication/308570348_Inverse_kinematic_implementation_of_four-wheels_mecanum_drive_mobile_robot_using_stepper_motors)
     * */
    fun inverse(velRobot: Twist2dDual, a: Double, b: Double): MecanumVelocitiesDual {
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
}

object DifferentialKinematics {
    /**
     * Gets the distance traveled along the X axis.
     * @param rIncr The distance increment measured by the right wheel.
     * @param rY The Y offset of the right wheel, from the center axis.
     * @param lIncr The distance increment measured by the left wheel.
     * @param lY The Y offset of the left wheel, from the center axis.
     * */
    fun xIncr(rIncr: Double, rY: Double, lIncr: Double, lY: Double): Double = (rIncr * lY - lIncr * rY) / (lY - rY)
    fun xIncrDual(rIncr: Dual, rY: Double, lIncr: Dual, lY: Double): Dual = (rIncr * lY - lIncr * rY) / (lY - rY)

    /**
     * Gets the angle increment.
     * @param rIncr The distance increment measured by the right wheel.
     * @param rY The Y offset of the right wheel, from the center axis.
     * @param lIncr The distance increment measured by the left wheel.
     * @param lY The Y offset of the left wheel, from the center axis.
     * */
    fun angleIncr(rIncr: Double, rY: Double, lIncr: Double, lY: Double): Double = (rIncr - lIncr) / (lY - rY)
    fun angleIncrDual(rIncr: Dual, rY: Double, lIncr: Dual, lY: Double): Dual = (rIncr - lIncr) / (lY - rY)
}

object Odometry {
    fun holo3WheelIncr(rIncr: Double, rY: Double, lIncr: Double, lY: Double, cIncr: Double, cX: Double): Twist2dIncr {
        val rotIncr = DifferentialKinematics.angleIncr(rIncr, rY, lIncr, lY)
        return Twist2dIncr(
            xIncr = DifferentialKinematics.xIncr(rIncr, rY, lIncr, lY),
            yIncr = cIncr - rotIncr * cX,
            rotIncr)
    }

    fun holo3WheelIncrDual(rIncr: Dual, rY: Double, lIncr: Dual, lY: Double, cIncr: Dual, cX: Double): Twist2dIncrDual {
        val rotIncr = DifferentialKinematics.angleIncrDual(rIncr, rY, lIncr, lY)
        return Twist2dIncrDual(
            xIncr = DifferentialKinematics.xIncrDual(rIncr, rY, lIncr, lY),
            yIncr = cIncr - rotIncr * cX,
            rotIncr)
    }
}