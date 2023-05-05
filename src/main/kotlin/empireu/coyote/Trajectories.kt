package empireu.coyote

import kotlin.math.absoluteValue
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

data class BaseTrajectoryConstraints(
    val linearVelocity: Double,
    val linearAcceleration: Double,
    val angularVelocity: Double,
    val angularAcceleration: Double,
    val centripetalAcceleration: Double
) {
    init {
        require(linearVelocity > 0.0) { "Linear velocity must be positive." }
        require(linearAcceleration > 0.0) { "Linear acceleration must be positive." }
        require(angularVelocity > 0.0) { "Angular velocity must be positive." }
        require(angularAcceleration > 0.0) { "Angular acceleration must be positive." }
        require(centripetalAcceleration > 0.0) { "Centripetal acceleration must be positive." }
    }
}

data class TrajectoryPoint(val curvePose: CurvePose2d) {
    var rotationCurvature = 0.0
    var displacement = 0.0
    var time = 0.0

    var velocity = Vector2d.zero
    var acceleration = Vector2d.zero
    var angularVelocity = 0.0
    var angularAcceleration = 0.0

    val pose get() = curvePose.pose

    fun copy(): TrajectoryPoint {
        return TrajectoryPoint(curvePose).also {
            it.rotationCurvature = rotationCurvature
            it.displacement = displacement
            it.time = time
            it.velocity = velocity
            it.acceleration = acceleration
            it.angularVelocity = angularVelocity
            it.angularAcceleration = angularAcceleration
        }
    }
}

object TrajectoryGenerator {
    private data class Intermediary(
        var linearDisplacement: Double,
        var linearVelocity: Double,
        var rotationCurvature: Double
    )

    /**
     * Assigns displacement and rotational curvature to the path points.
     * */
    private fun assignPathPoints(points: List<TrajectoryPoint>) {
        points[0].displacement = 0.0

        for (i in 1 until points.size) {
            val previous = points[i - 1]
            val current = points[i]

            current.displacement = previous.displacement + (current.pose.translation - previous.pose.translation).length

            if (current.displacement == previous.displacement) {
                error("Path points with zero displacement are not allowed")
            }

            current.rotationCurvature = (current.pose.rotation / previous.pose.rotation).log() / (current.displacement - previous.displacement)
        }
    }

    private fun computeMovementTime(
        displacement: Double,
        initialVelocity: Double,
        finalVelocity: Double,
        minAcceleration: Double,
        maxAcceleration: Double
    ): Double {
        require(minAcceleration < 0)
        require(maxAcceleration > 0)

        if (displacement == 0.0) {
            if (initialVelocity == finalVelocity) {
                return 0.0
            }

            error("Generation failed")
        }

        val acceleration = (finalVelocity.sqr() - initialVelocity.sqr()) / (displacement * 2.0)

        if (acceleration.absoluteValue > 0.0) {
            return (finalVelocity - initialVelocity) / acceleration
        }

        val vSum = initialVelocity + finalVelocity

        if (vSum == 0.0) {
            error("Invalid velocity set $initialVelocity $finalVelocity")
        }

        return 2.0 * displacement / vSum
    }

    private fun computeVelocityAcceleration(points: List<TrajectoryPoint>) {
        points[0].velocity = Vector2d.zero
        points[0].angularVelocity = 0.0

        for (i in 1 until points.size) {
            val previous = points[i - 1]
            val current = points[i]

            val dt = current.time - previous.time

            current.velocity = (current.curvePose.pose.translation - previous.curvePose.pose.translation) / dt
            current.angularVelocity = (current.curvePose.pose.rotation / previous.curvePose.pose.rotation).log().absoluteValue / dt
            current.acceleration = (current.velocity - previous.velocity) / dt
            current.angularAcceleration = (current.angularVelocity - previous.angularVelocity) / dt
        }
    }

    private fun computeUpperVelocities(
        trajectory: List<TrajectoryPoint>,
        profile: List<Intermediary>,
        constraints: BaseTrajectoryConstraints
    ) {
        // Angular Velocity:
        profile.forEach {
            it.linearVelocity = it.linearVelocity.minWith(
                constraints.angularVelocity / it.rotationCurvature.absoluteValue
            )
        }

        // Centripetal Acceleration:
        profile.forEachIndexed { index, it ->
            it.linearVelocity = it.linearVelocity.minWith(
                sqrt(constraints.centripetalAcceleration / trajectory[index].curvePose.curvature.absoluteValue)
            )
        }

        val awMax = constraints.angularAcceleration
        val atMax = constraints.linearAcceleration

        fun bounds(index1: Int, index: Int): Double {
            fun sqr(a: Double) = a * a

            val ci = profile[index].rotationCurvature
            val ci1 = profile[index1].rotationCurvature
            val ds = (trajectory[index].displacement - trajectory[index1].displacement).absoluteValue

            if (ds == 0.0) {
                error("Displacement is zero")
            }

            var thresh: Double = constraints.linearVelocity

            fun unexpectedSet() {
                error("Unexpected $ci $ci1")
            }

            if (ci > 0.0 && ci1 >= 0.0) {
                if (ci > ci1) {
                    thresh = sqrt(
                        2.0 * ds * sqr(awMax + ci * atMax) /
                                ((atMax * (ci + ci1) + 2.0 * awMax) * (ci - ci1))
                    )

                    thresh.throwIfNan()
                } else if (ci < ci1) {
                    val thresh1 = sqrt(8.0 * ci * awMax * ds / sqr(ci1 + ci))
                    val tmp1 = sqrt(4.0 * ci * ds * (ci * atMax + awMax) / sqr(ci1 - ci))
                    val tmp2 = sqrt(
                        2.0 * ds * sqr(ci * atMax + awMax) /
                                ((ci1 - ci) * (2.0 * awMax + (ci1 + ci) * atMax))
                    )
                    val threshTmp1 = min(tmp1, tmp2)
                    val threshTmp2 = min(sqrt(2 * awMax * ds / ci1), sqrt(2 * atMax * ds))
                    var threshTmp3 = Double.NEGATIVE_INFINITY
                    val tmp = min(
                        2.0 * awMax * ds / ci1,
                        2.0 * ds * sqr(ci * atMax - awMax) / ((ci1 - ci) * (2.0 * awMax - (ci1 + ci) * atMax))
                    )

                    if (tmp > -4.0 * ci * ds * (ci * atMax - awMax) / ((ci1 - ci) * (ci1 + ci)) && tmp > 2.0 * atMax * ds) {
                        threshTmp3 = sqrt(tmp)
                    }

                    thresh = max(max(thresh1, threshTmp1), max(threshTmp2, threshTmp3))

                    thresh.throwIfNan()
                } else if (ci == ci1) {
                    thresh = Double.POSITIVE_INFINITY
                } else {
                    unexpectedSet()
                }
            } else if (ci < 0.0 && ci1 <= 0.0) {
                if (ci > ci1) {
                    val thresh1 = sqrt(-8.0 * ci * awMax * ds / sqr(ci1 + ci))
                    val tmp1 =
                        sqrt(-4.0 * ci * ds * (awMax - ci * atMax) / ((ci1 + ci) * (ci1 - ci)))
                    val tmp2 = sqrt(
                        -2.0 * ds * sqr(awMax - ci * atMax) /
                                ((ci1 - ci) * (2 * awMax - (ci1 + ci) * atMax))
                    )

                    val threshTmp1 = min(tmp1, tmp2)
                    val threshTmp2 = min(sqrt(-2 * awMax * ds / ci1), sqrt(2 * atMax * ds))
                    var threshTmp3 = Double.NEGATIVE_INFINITY

                    val tmp = min(
                        -2.0 * awMax * ds / ci1,
                        -2.0 * ds * sqr(ci * atMax - awMax) / ((ci1 - ci) * (2.0 * awMax + (ci1 + ci) * atMax))
                    )

                    if (tmp > -4.0 * ci * ds * (awMax + ci * atMax) / ((ci1 - ci) * (ci1 + ci)) && tmp > 2.0 * atMax * ds) {
                        threshTmp3 = sqrt(tmp)
                    }

                    thresh = max(max(thresh1, threshTmp1), max(threshTmp2, threshTmp3))
                    thresh.throwIfNan()
                } else if (ci < ci1) {
                    thresh = sqrt(
                        -2.0 * ds * sqr(awMax - ci * atMax) / ((ci1 - ci) * ((ci + ci1) * atMax - 2.0 * awMax))
                    )
                    thresh.throwIfNan()
                } else if (ci == ci1) {
                    thresh = Double.POSITIVE_INFINITY
                } else {
                    unexpectedSet()
                }
            } else if (ci < 0.0 && ci1 > 0.0) {
                val vtWoStarPos = sqrt(2.0 * ds * awMax / ci1)
                var precond = Double.POSITIVE_INFINITY
                if (ci1 + ci < 0) {
                    precond =
                        sqrt(-4.0 * ci * ds * (ci * atMax - awMax) / ((ci1 - ci) * (ci1 + ci)))
                }
                var threshTmp = min(
                    precond,
                    sqrt(-2.0 * ds * sqr(ci * atMax - awMax) / ((ci1 - ci) * ((ci1 + ci) * atMax - 2.0 * awMax)))
                )
                threshTmp = max(threshTmp, sqrt(2 * ds * atMax))
                thresh = min(threshTmp, vtWoStarPos)
                thresh.throwIfNan()
            } else if (ci > 0.0 && ci1 < 0.0) {
                val v1StarPos = sqrt(-(2.0 * ds * awMax / ci1))
                var precond = Double.POSITIVE_INFINITY

                if (ci1 + ci > 0.0) {
                    precond =
                        sqrt(-4.0 * ci * ds * (awMax + ci * atMax) / ((ci1 - ci) * (ci1 + ci)))
                }

                var threshTmp: Double = min(
                    precond,
                    sqrt(-2.0 * ds * sqr(awMax + ci * atMax) / ((ci1 - ci) * ((ci1 + ci) * atMax + 2.0 * awMax)))
                )
                threshTmp = max(threshTmp, sqrt(2.0 * ds * atMax))
                thresh = min(threshTmp, v1StarPos)
                thresh.throwIfNan()
            } else if (ci == 0.0 && ci1 == 0.0) {
                thresh = Double.POSITIVE_INFINITY
            } else if (ci == 0.0) {
                if (ci1 > 0.0) {
                    val vtWoHatPos = sqrt(2.0 * ds * awMax / ci1)
                    val threshTmp = maxNaN(
                        sqrt(2.0 * ds * atMax),
                        sqrt(-2.0 * ds * sqr(awMax) / (ci1 * (ci1 * atMax - 2.0 * awMax)))
                    )
                    thresh = minNaN(vtWoHatPos, threshTmp).throwIfNan()
                } else if (ci1 < 0.0) {
                    val v1HatPos = sqrt(-(2.0 * ds * awMax / ci1))
                    val threshTmp = maxNaN(
                        sqrt(2.0 * ds * atMax),
                        sqrt(-2.0 * ds * sqr(awMax) / (ci1 * (ci1 * atMax + 2.0 * awMax)))
                    )
                    thresh = minNaN(v1HatPos, threshTmp).throwIfNan()
                } else {
                    unexpectedSet()
                }
            } else {
                unexpectedSet()
            }

            return thresh.throwIfNan()
        }

        // Angular acceleration:

        profile.first().linearVelocity = 0.0
        for (i in 1 until profile.size) {
            profile[i - 1].linearVelocity = profile[i - 1].linearVelocity.minWith(
                bounds(i - 1, i)
            )
        }

        profile.last().linearVelocity = 0.0
        for (i in trajectory.size - 2 downTo 0) {
            profile[i + 1].linearVelocity = profile[i + 1].linearVelocity.minWith(
                bounds(i + 1, i)
            )
        }
    }

    fun generateProfile(path: List<CurvePose2d>, constraints: BaseTrajectoryConstraints): ArrayList<TrajectoryPoint> {
        require(path.size >= 2)

        val points = ArrayList<TrajectoryPoint>(path.size)

        path.forEach {
            points.add(TrajectoryPoint(it))
        }

        assignPathPoints(points)

        val profile = ArrayList<Intermediary>(path.size)

        points.forEach {
            profile.add(
                Intermediary(
                    it.displacement,
                    constraints.linearVelocity,
                    it.rotationCurvature
                )
            )
        }

        computeUpperVelocities(points, profile, constraints)

        fun combinedPass(previousIndex: Int, currentIndex: Int) {
            val pi1 = profile[previousIndex]
            val pi = profile[currentIndex]

            val atMax = constraints.linearAcceleration
            val awMax = constraints.angularAcceleration

            val ci1 = profile[previousIndex].rotationCurvature
            val ci = profile[currentIndex].rotationCurvature

            val vi1 = pi1.linearVelocity

            val ds = (pi.linearDisplacement - pi1.linearDisplacement).absoluteValue

            val vtAt = Range(
                if (vi1.sqr() > 2.0 * atMax * ds) sqrt(vi1.sqr() - 2.0 * atMax * ds)
                else 0.0,
                sqrt(vi1.sqr() + 2.0 * atMax * ds)
            )

            //#region Analysis

            fun v1() = 1.0 / (2.0 * ci) * ((ci1 - ci) * vi1 + sqrt((ci + ci1).sqr() * vi1.sqr() + 8.0 * ci * ds * awMax))
            fun v2() = 1.0 / (2.0 * ci) * ((ci1 - ci) * vi1 - sqrt((ci + ci1).sqr() * vi1.sqr() + 8.0 * ci * ds * awMax))
            fun v1Star() = 1.0 / (2.0 * ci) * ((ci1 - ci) * vi1 + sqrt((ci + ci1).sqr() * vi1.sqr() - 8.0 * ci * ds * awMax))
            fun v2Star() = 1.0 / (2.0 * ci) * ((ci1 - ci) * vi1 - sqrt((ci + ci1).sqr() * vi1.sqr() - 8.0 * ci * ds * awMax))
            fun v1Hat() = -(2.0 * ds * awMax) / (ci1 * vi1) - vi1
            fun v2Hat() = 2.0 * ds * awMax / (ci1 * vi1) - vi1

            //#endregion

            val vtAwRanges = ArrayList<Range>(2)

            if (ci > 0) {
                val condition = (ci + ci1).sqr() * vi1.sqr() - 8.0 * ci * awMax * ds

                if (condition < 0) {
                    vtAwRanges.add(Range(v2(), v1()))
                } else {
                    require(condition >= 0)

                    vtAwRanges.add(Range(v2(), v2Star()))
                    vtAwRanges.add(Range(v1Star(), v1()))
                }
            } else if (ci < 0) {
                val condition = (ci + ci1).sqr() * vi1.sqr() + 8 * ci * awMax * ds

                if (condition < 0) {
                    vtAwRanges.add(Range(v1Star(), v2Star()))
                } else {
                    require(condition >= 0)

                    vtAwRanges.add(Range(v1Star(), v1()))
                    vtAwRanges.add(Range(v2(), v2Star()))
                }
            } else {
                require(ci == 0.0)

                if (ci1 > 0) {
                    vtAwRanges.add(Range(v1Hat(), v2Hat()))
                } else if (ci1 < 0) {
                    vtAwRanges.add(Range(v2Hat(), v1Hat()))
                } else {
                    vtAwRanges.add(Range.R)
                }
            }

            var velocity = 0.0

            vtAwRanges.forEachIndexed { index, vtAw ->
                val intersect = intersect(vtAw, vtAt)

                if (!intersect.isValid) {
                    if (index == vtAwRanges.size - 1) {
                        val closest = vtAwRanges
                            .flatMap { listOf(it.min, it.max) }
                            .filter { it >= -10e-4 }
                            .flatMap { av ->
                                listOf(
                                    Pair(av, vtAt.min),
                                    Pair(av, vtAt.max)
                                )
                            }
                            .minBy { kotlin.math.abs(it.first - it.second) }

                        velocity = max((closest.first + closest.second) / 2.0, 0.0)
                    }

                    return@forEachIndexed
                }

                velocity = max(velocity, intersect.max)
            }

            if (!velocity.isFinite()) {
                error("Failed to find velocity")
            }

            pi.linearVelocity = pi.linearVelocity.minWith(velocity)
        }

        // Forward pass:
        profile.first().linearVelocity = 0.0
        for (i in 1 until profile.size) {
            combinedPass(i - 1, i)
        }

        // Backward pass:
        profile.last().linearVelocity = 0.0
        for (i in profile.size - 2 downTo 0) {
            combinedPass(i + 1, i)
        }

        for (i in 1 until profile.size) {
            val previous = profile[i - 1]
            val current = profile[i]

            points[i].time = points[i - 1].time + computeMovementTime(
                current.linearDisplacement - previous.linearDisplacement,
                previous.linearVelocity,
                current.linearVelocity,
                -constraints.linearAcceleration,
                constraints.linearAcceleration
            )
        }

        computeVelocityAcceleration(points)

        return points
    }

    fun generateTrajectory(
        path: List<CurvePose2d>,
        constraints: BaseTrajectoryConstraints
    ): Trajectory {
        return Trajectory(generateProfile(path, constraints))
    }
}

class Trajectory(val source: List<TrajectoryPoint>) {
    private data class Segment(val a: TrajectoryPoint, val b: TrajectoryPoint) {
        fun evaluate(t: Double): TrajectoryPoint {
            if (t < a.time || t > b.time) {
                error("Invalid time query $t")
            }

            val progress = t.mappedTo(a.time, b.time, 0.0, 1.0)

            return TrajectoryPoint(
                CurvePose2d(
                    interpolate(a.curvePose.pose, b.curvePose.pose, progress),
                    lerp(a.curvePose.curvature, b.curvePose.curvature, progress),
                )
            ).also {
                it.rotationCurvature = lerp(a.rotationCurvature, b.rotationCurvature, progress)
                it.displacement = lerp(a.displacement, b.displacement, progress)
                it.time = lerp(a.time, b.time, progress) // equal to t, actually
                it.velocity = lerp(a.velocity, b.velocity, progress)
                it.acceleration = lerp(a.acceleration, b.acceleration, progress)
                it.angularVelocity = lerp(a.angularVelocity, b.angularVelocity, progress)
                it.angularAcceleration = lerp(a.angularAcceleration, b.angularAcceleration, progress)
            }
        }
    }

    private val segments: SegmentTree<Segment>

    val points get() = source.size

    init {
        require(source.size >= 2)

        val builder = SegmentTreeBuilder<Segment>()

        for (i in 1 until source.size) {
            val previous = source[i - 1]
            val current = source[i]

            builder.insert(
                Segment(previous, current),
                SegmentRange(previous.time, current.time)
            )
        }

        segments = builder.build()
    }

    val timeStart get() = segments.range.start
    val timeEnd get() = segments.range.end
    val duration get() = timeEnd - timeStart

    fun evaluate(t: Double): TrajectoryPoint {
        val time = t.coerceIn(timeStart, timeEnd)

        return segments
            .query(time)
            .evaluate(time)
    }

    fun inverse(): Trajectory {
        val reversePoints = ArrayList<TrajectoryPoint>(source.size)

        for (i in source.size - 1 downTo 0) {
            reversePoints.add(source[i].copy().also { copy ->
                copy.time = timeEnd - copy.time
                copy.velocity = -copy.velocity
                copy.acceleration = -copy.acceleration
                copy.angularVelocity = -copy.angularVelocity
                copy.angularAcceleration = -copy.angularAcceleration

                // Maybe invert the others?
            })
        }

        return Trajectory(reversePoints)
    }
}
