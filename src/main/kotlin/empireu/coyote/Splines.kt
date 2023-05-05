package empireu.coyote

import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sqrt

object Splines {
    data class SplineIndices(val index: Int, val t: Double)

    fun getUniformIndices(segments: Int, t: Double): SplineIndices {
        var progress = t.coerceIn(0.0, 1.0)

        progress *= segments.toDouble()

        val index = progress.toInt().coerceIn(0, segments - 1)

        return SplineIndices(index, progress - index)
    }

    fun hermiteQuintic(p0: Double, v0: Double, a0: Double, a1: Double, v1: Double, p1: Double, t: Double): Double {
        val t2 = t * t
        val t3 = t2 * t
        val t4 = t3 * t
        val t5 = t4 * t
        val h0 = 1.0 - 10.0 * t3 + 15.0 * t4 - 6.0 * t5
        val h1 = t - 6.0 * t3 + 8.0 * t4 - 3.0 * t5
        val h2 = 1.0 / 2.0 * t2 - 3.0 / 2.0 * t3 + 3.0 / 2.0 * t4 - 1.0 / 2.0 * t5
        val h3 = 1.0 / 2.0 * t3 - t4 + 1.0 / 2.0 * t5
        val h4 = -4.0 * t3 + 7.0 * t4 - 3.0 * t5
        val h5 = 10.0 * t3 - 15.0 * t4 + 6.0 * t5

        return h0 * p0 + h1 * v0 + h2 * a0 + h3 * a1 + h4 * v1 + h5 * p1
    }

    fun hermiteQuinticDerivative1(p0: Double, v0: Double, a0: Double, a1: Double, v1: Double, p1: Double, t: Double): Double {
        val t2 = t * t
        val h0 = -30.0 * ((t - 1.0) * (t - 1.0)) * t2
        val h1 = -((t - 1.0) * (t - 1.0)) * (15.0 * t2 - 2.0 * t - 1.0)
        val h2 = -1.0 / 2.0 * ((t - 1.0) * (t - 1.0)) * t * (5.0 * t - 2.0)
        val h3 = 1.0 / 2.0 * t2 * (5.0 * t2 - 8.0 * t + 3.0)
        val h4 = t2 * (-15.0 * t2 + 28.0 * t - 12.0)
        val h5 = 30.0 * ((t - 1.0) * (t - 1.0)) * t2

        return h0 * p0 + h1 * v0 + h2 * a0 + h3 * a1 + h4 * v1 + h5 * p1
    }

    fun hermiteQuinticDerivative2(p0: Double, v0: Double, a0: Double, a1: Double, v1: Double, p1: Double, t: Double): Double {
        val t2 = t * t
        val t3 = t2 * t
        val h0 = -60.0 * t * (2.0 * t2 - 3.0 * t + 1.0)
        val h1 = -12.0 * t * (5.0 * t2 - 8.0 * t + 3.0)
        val h2 = -10.0 * t3 + 18.0 * t2 - 9.0 * t + 1.0
        val h3 = t * (10.0 * t2 - 12.0 * t + 3.0)
        val h4 = -12.0 * t * (5.0 * t2 - 7.0 * t + 2.0)
        val h5 = 60.0 * t * (2.0 * t2 - 3.0 * t + 1.0)

        return h0 * p0 + h1 * v0 + h2 * a0 + h3 * a1 + h4 * v1 + h5 * p1
    }

    fun hermiteQuintic2d(p0: Vector2d, v0: Vector2d, a0: Vector2d, a1: Vector2d, v1: Vector2d, p1: Vector2d, t: Double): Vector2d {
        return Vector2d(
            hermiteQuintic(p0.x, v0.x, a0.x, a1.x, v1.x, p1.x, t),
            hermiteQuintic(p0.y, v0.y, a0.y, a1.y, v1.y, p1.y, t)
        )
    }

    fun hermiteQuintic2dDerivative1(p0: Vector2d, v0: Vector2d, a0: Vector2d, a1: Vector2d, v1: Vector2d, p1: Vector2d, t: Double): Vector2d {
        return Vector2d(
            hermiteQuinticDerivative1(p0.x, v0.x, a0.x, a1.x, v1.x, p1.x, t),
            hermiteQuinticDerivative1(p0.y, v0.y, a0.y, a1.y, v1.y, p1.y, t)
        )
    }

    fun hermiteQuintic2dDerivative2(p0: Vector2d, v0: Vector2d, a0: Vector2d, a1: Vector2d, v1: Vector2d, p1: Vector2d, t: Double): Vector2d {
        return Vector2d(
            hermiteQuinticDerivative2(p0.x, v0.x, a0.x, a1.x, v1.x, p1.x, t),
            hermiteQuinticDerivative2(p0.y, v0.y, a0.y, a1.y, v1.y, p1.y, t)
        )
    }

    fun hermiteQuintic2dCurvature(p0: Vector2d, v0: Vector2d, a0: Vector2d, a1: Vector2d, v1: Vector2d, p1: Vector2d, t: Double): Double {
        val d = hermiteQuintic2dDerivative1(p0, v0, a0, a1, v1, p1, t)
        val dd = hermiteQuintic2dDerivative2(p0, v0, a0, a1, v1, p1, t)

        val dx = d.x
        val dy = d.y
        val ddx = dd.x
        val ddy = dd.y

        return (dx * ddy - ddx * dy) / ((dx * dx + dy * dy) * sqrt((dx * dx + dy * dy)))
    }

    fun hermiteQuinticKd(p0: VectorKd, v0: VectorKd, a0: VectorKd, a1: VectorKd, v1: VectorKd, p1: VectorKd, t: Double): VectorKd {
        validate(p0, v0, a0, a1, v1, p1)

        val results = DoubleArray(p0.size)

        for (i in results.indices) {
            results[i] = hermiteQuintic(p0[i], v0[i], a0[i], a1[i], v1[i], p1[i], t)
        }

        return VectorKd(results)
    }

    fun hermiteQuinticKdDerivative1(p0: VectorKd, v0: VectorKd, a0: VectorKd, a1: VectorKd, v1: VectorKd, p1: VectorKd, t: Double): VectorKd {
        validate(p0, v0, a0, a1, v1, p1)

        val results = DoubleArray(p0.size)

        for (i in results.indices) {
            results[i] = hermiteQuinticDerivative1(p0[i], v0[i], a0[i], a1[i], v1[i], p1[i], t)
        }

        return VectorKd(results)
    }

    fun hermiteQuinticKdDerivative2(p0: VectorKd, v0: VectorKd, a0: VectorKd, a1: VectorKd, v1: VectorKd, p1: VectorKd, t: Double): VectorKd {
        validate(p0, v0, a0, a1, v1, p1)

        val results = DoubleArray(p0.size)

        for (i in results.indices) {
            results[i] = hermiteQuinticDerivative2(p0[i], v0[i], a0[i], a1[i], v1[i], p1[i], t)
        }

        return VectorKd(results)
    }
}

interface IPositionSpline<TParameter> {
    fun evaluatePosition(t: Double): VectorKd
}

interface IVelocitySpline<TParameter> {
    fun evaluateVelocity(t: Double): VectorKd
}

interface IAccelerationSpline<TParameter> {
    fun evaluateAcceleration(t: Double): VectorKd
}

interface IPoseSpline<TParameter> {
    fun evaluatePose(t: Double): Pose2d
}

interface ICurvatureSpline<TParameter> {
    fun evaluateCurvature(t: Double): Double
}

interface ICurvePoseSpline<TParameter> {
    fun evaluateCurvePose(t: Double): CurvePose2d
}

interface IKDSpline {
    val size: Int
}

data class QuinticSplineSegment(val p0: VectorKd, val v0: VectorKd, val a0: VectorKd, val a1: VectorKd, val v1: VectorKd, val p1: VectorKd) {
    init {
        validate(p0, v0, a0, a1, v1, p1)
    }

    val size get() = p0.size

    fun evaluatePosition(t: Double): VectorKd = Splines.hermiteQuinticKd(p0, v0, a0, a1, v1, p1, t)
    fun evaluateVelocity(t: Double): VectorKd = Splines.hermiteQuinticKdDerivative1(p0, v0, a0, a1, v1, p1, t)
    fun evaluateAcceleration(t: Double): VectorKd = Splines.hermiteQuinticKdDerivative2(p0, v0, a0, a1, v1, p1, t)

    fun evaluateCurvature(t: Double): Double {
        if (size != 2) {
            error("Cannot evaluate curvature of ${size}D spline")
        }

        return Splines.hermiteQuintic2dCurvature(
            p0.toVector2d(),
            v0.toVector2d(),
            a0.toVector2d(),
            a1.toVector2d(),
            v1.toVector2d(),
            p1.toVector2d(),
            t
        )
    }
}

fun validateSpline(segments: List<QuinticSplineSegment>) {
    require(segments.isNotEmpty())

    if (segments.size == 1) {
        return
    }

    for (i in 1 until segments.size) {
        if (segments[i - 1].p1 != segments[i].p0) {
            error("Spline segment discontinuity")
        }

        if (segments[i - 1].size != segments[i].size) {
            error("Mismatched spline segment component count")
        }
    }
}

private fun IKDSpline.require2() {
    if (this.size != 2) {
        error("This operation requires a spline of size 2")
    }
}

interface Percentage

class QuinticSpline(private val segments: List<QuinticSplineSegment>) :
    IPositionSpline<Percentage>,
    IVelocitySpline<Percentage>,
    IAccelerationSpline<Percentage>,
    IPoseSpline<Percentage>,
    ICurvatureSpline<Percentage>,
    ICurvePoseSpline<Percentage>,
    IKDSpline {
    init {
        validateSpline(segments)
    }

    override val size get() = segments[0].size

    override fun evaluatePosition(t: Double): VectorKd {
        val indices = Splines.getUniformIndices(segments.size, t)

        return segments[indices.index].evaluatePosition(indices.t)
    }

    override fun evaluateVelocity(t: Double): VectorKd {
        val indices = Splines.getUniformIndices(segments.size, t)

        return segments[indices.index].evaluateVelocity(indices.t)
    }

    override fun evaluateAcceleration(t: Double): VectorKd {
        val indices = Splines.getUniformIndices(segments.size, t)

        return segments[indices.index].evaluateAcceleration(indices.t)
    }

    override fun evaluatePose(t: Double): Pose2d {
        require2()

        return Pose2d(
            evaluatePosition(t).toVector2d(),
            Rotation2d.dir(evaluateVelocity(t).toVector2d())
        )
    }

    override fun evaluateCurvature(t: Double): Double {
        require2()

        val indices = Splines.getUniformIndices(segments.size, t)

        return segments[indices.index].evaluateCurvature(indices.t)
    }

    override fun evaluateCurvePose(t: Double): CurvePose2d {
        require2()

        return CurvePose2d(evaluatePose(t), evaluateCurvature(t))
    }

    fun computeArcLength(points: Int = 1024): Double {
        require(points >= 2)

        var result = 0.0
        val sampleSize = 1.0 / points

        for (i in 1 until points) {
            val t0 = (i - 1.0) * sampleSize
            val t1 = t0 + sampleSize

            result += distance(evaluatePosition(t0), evaluatePosition(t1))
        }

        return result
    }
}


data class QuinticSplineSegmentMapped(val keyRange: Range, val spline: QuinticSplineSegment)

interface MappedPercentage

/**
 * Represents a piecewise Quintic Hermite Spline with arbitrary parameter.
 * */
class QuinticSplineMapped(val segments: List<QuinticSplineSegmentMapped>) :
    IPositionSpline<MappedPercentage>,
    IVelocitySpline<MappedPercentage>,
    IAccelerationSpline<MappedPercentage>,
    IPoseSpline<MappedPercentage>,
    ICurvatureSpline<MappedPercentage>,
    ICurvePoseSpline<MappedPercentage>,
    IKDSpline {
    init {
        validateSpline(segments.map { it.spline })

        if (segments.any { !it.keyRange.isValid }) {
            error("Invalid spline range")
        }

        if (segments.size > 1) {
            for (i in 1 until segments.size) {
                if (segments[i - 1].keyRange.max != segments[i].keyRange.min) {
                    error("Spline key discontinuity")
                }
            }
        }
    }

    override val size get() = segments[0].spline.size

    /**
     * Gets the lower bound of the parameter range.
     * */
    val startKey get() = segments.first().keyRange.min

    /**
     * Gets the upper bound of the parameter range.
     * */
    val endKey get() = segments.last().keyRange.max

    /**
     * Gets the parameter range.
     * */
    val range get() = Range(startKey, endKey)

    private data class Point(val segment: QuinticSplineSegmentMapped, val parameter: Double)

    private fun getPoint(key: Double): Point {
        if (key <= startKey) {
            return Point(segments.first(), 0.0)
        }

        if (key >= endKey) {
            return Point(segments.last(), 1.0)
        }

        // We have pretty small splines, but maybe replace with search or segment tree?

        // This search should succeed because we checked continuities, and we have those conditions at the start.
        val segment = segments.first { it.keyRange.min <= key && it.keyRange.max >= key }

        val progress = map(
            key,
            segment.keyRange.min,
            segment.keyRange.max,
            0.0,
            1.0
        )

        return Point(segment, progress)
    }

    override fun evaluatePosition(t: Double): VectorKd {
        val point = getPoint(t)

        return point.segment.spline.evaluatePosition(point.parameter)
    }

    override fun evaluateVelocity(t: Double): VectorKd {
        val point = getPoint(t)

        return point.segment.spline.evaluateVelocity(point.parameter)
    }

    override fun evaluateAcceleration(t: Double): VectorKd {
        val point = getPoint(t)

        return point.segment.spline.evaluateAcceleration(point.parameter)
    }

    override fun evaluatePose(t: Double): Pose2d {
        require2()

        return Pose2d(
            evaluatePosition(t).toVector2d(),
            Rotation2d.dir(evaluateVelocity(t).toVector2d())
        )
    }

    override fun evaluateCurvature(t: Double): Double {
        require2()

        val point = getPoint(t)

        return point.segment.spline.evaluateCurvature(point.parameter)
    }

    override fun evaluateCurvePose(t: Double): CurvePose2d {
        require2()

        return CurvePose2d(evaluatePose(t), evaluateCurvature(t))
    }
}

class QuinticSplineMappedBuilder(val size: Int) {
    init {
        require(size > 0)
    }

    private data class SplinePoint(
        val key: Double,
        val displacement: VectorKd,
        val velocity: VectorKd,
        val acceleration: VectorKd
    )

    private val points = ArrayList<SplinePoint>()

    fun add(key: Double, displacement: VectorKd, velocity: VectorKd, acceleration: VectorKd) {
        validate(displacement, size)
        validate(velocity, size)
        validate(acceleration, size)

        if (points.isNotEmpty()) {
            if (key <= points.last().key) {
                error("Key continuity broken")
            }
        }

        points.add(
            SplinePoint(
                key,
                displacement,
                velocity,
                acceleration
            )
        )
    }

    val canBuild get() = points.size > 1

    fun build(): QuinticSplineMapped {
        require(canBuild)

        fun get(index: Int): SplinePoint {
            if (index < 0) {
                return points.first()
            }

            if (index >= points.size) {
                return points.last()
            }

            return points[index]
        }

        val results = ArrayList<QuinticSplineSegmentMapped>(points.size - 1)

        for (i in 1 until points.size) {
            val a = get(i - 1)
            val b = get(i)

            results.add(
                QuinticSplineSegmentMapped(
                    Range(a.key, b.key),
                    QuinticSplineSegment(
                        a.displacement,
                        a.velocity,
                        a.acceleration,
                        b.acceleration,
                        b.velocity,
                        b.displacement
                    )
                )
            )
        }

        return QuinticSplineMapped(results)
    }
}

/**
 * Projects the given [position] onto the spline using an initial rough estimate, found by sampling [projectionSamples] points and
 * selecting the closest one, then optimizing the estimate using "Binary Descent" in [descentSteps] steps using the specified
 * hyper-parameter [descentFalloff].
 *
 * @return The internal spline parameter whose position is closest to [position].
 * */
fun IPositionSpline<Percentage>.project(
    position: VectorKd,
    projectionSamples: Int = 128,
    descentSteps: Int = 32,
    descentFalloff: Double = 1.25
) : Double {
    require(projectionSamples > 0)
    require(descentSteps > 1)

    var closest = 0.0
    var closestDistance = Double.MAX_VALUE

    for (sample in 0 until projectionSamples) {
        val t = sample / (projectionSamples - 1.0)
        val point = this.evaluatePosition(t)
        val distance = distanceSqr(position, point)

        if (distance < closestDistance) {
            closest = t
            closestDistance = distance
        }
    }

    fun projectError(t: Double): Double = distanceSqr(this.evaluatePosition(t.coerceIn(0.0, 1.0)), position)

    if (closest > 0 && closest < 1) {
        var descentRate = 1.0 / projectionSamples

        repeat(descentSteps) {
            val errorLeft = projectError(closest - descentRate)
            val errorRight = projectError(closest + descentRate)

            var step = -descentRate
            var adjustedError = errorLeft

            if (errorRight < errorLeft) {
                step = descentRate
                adjustedError = errorRight
            }

            val currentError = projectError(closest)

            if (adjustedError > currentError) {
                descentRate = descentRate.pow(descentFalloff)
            } else {
                closest += step
            }
        }
    }

    return closest.coerceIn(0.0, 1.0)
}

private data class GetPointsFrame<TParameter>(val t0: Double, val t1: Double)

data class CurvePoseParam<TParameter>(val curvePose: CurvePose2d, val param: Double)

fun <TParameter> ICurvePoseSpline<TParameter>.getPointsWpi(
    results: MutableList<CurvePoseParam<TParameter>>,
    t0: Double,
    t1: Double,
    admissibleT: Double,
    admissibleIncr: Twist2dIncr,
    maxIterations: Int = Int.MAX_VALUE,
    splitCondition: ((Double, Double) -> Boolean)? = null
) {
    require(admissibleIncr.trIncr.x > 0.0) { "Admissible x increment must be positive" }
    require(admissibleIncr.trIncr.y > 0.0) { "Admissible y increment must be positive" }
    require(admissibleIncr.rotIncr > 0.0) { "Admissible rotation increment must be positive" }
    require(t0 < t1) { "Invalid parameters $t0 $t1" }

    results.add(CurvePoseParam(this.evaluateCurvePose(t0), t0))

    val stack = ArrayDeque<GetPointsFrame<TParameter>>()
    stack.addLast(GetPointsFrame(t0, t1))

    var iterations = 0
    var t = t0

    while (stack.isNotEmpty()) {
        val current = stack.removeLast()

        val start = this.evaluateCurvePose(current.t0)
        val end = this.evaluateCurvePose(current.t1)

        val incr = (start.pose / end.pose).log()

        if (abs(current.t1 - t) > admissibleT ||
            (splitCondition != null && splitCondition(current.t0, current.t1)) ||
            abs(incr.trIncr.x) > admissibleIncr.trIncr.x ||
            abs(incr.trIncr.y) > admissibleIncr.trIncr.y ||
            abs(incr.rotIncr) > admissibleIncr.rotIncr
        ) {
            stack.addLast(GetPointsFrame((current.t0 + current.t1) / 2.0, current.t1))
            stack.addLast(GetPointsFrame(current.t0, (current.t0 + current.t1) / 2.0))
        } else {
            results.add(CurvePoseParam(end, current.t1))
            t = current.t1
        }

        if (iterations++ >= maxIterations) {
            error("GetPoints exceeded max iterations")
        }
    }
}