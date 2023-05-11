package empireu.coyote

import kotlin.math.*

data class Vector2d(val x: Double, val y: Double) {
    constructor(value: Double): this(value, value)

    val lengthSqr get() = x * x + y * y
    val length get() = sqrt(lengthSqr)
    fun normalized() = this / length

    fun approxEqs(other: Vector2d, eps: Double = 10e-6) = x.approxEquals(other.x, eps) && y.approxEquals(other.y, eps)

    override fun toString(): String {
        return "x=${x.rounded()}, y=${y.rounded()}"
    }

    override fun equals(other: Any?): Boolean {
        if(other !is Vector2d) {
            return false
        }

        return equals(other)
    }

    fun equals(other: Vector2d) = (x == other.x && y == other.y)

    override fun hashCode(): Int {
        var result = x.hashCode()
        result = 31 * result + y.hashCode()
        return result
    }

    operator fun unaryPlus() = this
    operator fun unaryMinus() = Vector2d(-x, -y)
    operator fun plus(other: Vector2d) = Vector2d(x + other.x, y + other.y)
    operator fun minus(other: Vector2d) = Vector2d(x - other.x, y - other.y)
    operator fun times(other: Vector2d) = Vector2d(x * other.x, y * other.y)
    operator fun div(other: Vector2d) = Vector2d(x / other.x, y / other.y)
    operator fun times(scalar: Double) = Vector2d(x * scalar, y * scalar)
    operator fun div(scalar: Double) = Vector2d(x / scalar, y / scalar)

    operator fun compareTo(other: Vector2d) = this.lengthSqr.compareTo(other.lengthSqr)

    companion object {
        val zero = Vector2d(0.0, 0.0)
        val one = Vector2d(1.0, 1.0)
        val unitX = Vector2d(1.0, 0.0)
        val unitY = Vector2d(0.0, 1.0)
    }
}

fun lerp(a: Vector2d, b: Vector2d, t: Double) = Vector2d(
    lerp(a.x, b.x, t),
    lerp(a.y, b.y, t)
)

data class Vector2dDual(val x: Dual, val y: Dual) {
    constructor(value: Dual): this(value, value)

    init {
        require(x.size == y.size) { "Dual X and Y must be of the same size" }
        require(x.size > 0)
    }

    val size get() = x.size
    val isReal get() = size == 1
    val lengthSqr get() = x * x + y * y
    val length get() = sqrt(lengthSqr)
    fun normalized() = this / length
    val value get() = Vector2d(x.value, y.value)
    fun head(n: Int = 1) = Vector2dDual(x.head(n), y.head(n))
    fun tail(n: Int = 1) = Vector2dDual(x.tail(n), y.tail(n))

    override fun toString(): String {
        return "x=${x}, y=${y}"
    }

    override fun equals(other: Any?): Boolean {
        if(other !is Vector2dDual) {
            return false
        }

        return equals(other)
    }

    fun equals(other: Vector2dDual) = (x == other.x && y == other.y)

    override fun hashCode(): Int {
        var result = x.hashCode()
        result = 31 * result + y.hashCode()
        return result
    }

    operator fun unaryPlus() = this
    operator fun unaryMinus() = Vector2dDual(-x, -y)
    operator fun plus(other: Vector2dDual) = Vector2dDual(x + other.x, y + other.y)
    operator fun minus(other: Vector2dDual) = Vector2dDual(x - other.x, y - other.y)
    operator fun times(other: Vector2dDual) = Vector2dDual(x * other.x, y * other.y)
    operator fun div(other: Vector2dDual) = Vector2dDual(x / other.x, y / other.y)
    operator fun times(scalar: Dual) = Vector2dDual(x * scalar, y * scalar)
    operator fun div(scalar: Dual) = Vector2dDual(x / scalar, y / scalar)
    operator fun times(constant: Double) = Vector2dDual(x * constant, y * constant)
    operator fun div(constant: Double) = Vector2dDual(x / constant, y / constant)

    companion object {
        fun const(x: Double, y: Double, n: Int = 1) = Vector2dDual(Dual.const(x, n), Dual.const(y, n))
        fun const(value: Vector2d, n: Int = 1) = const(value.x, value.y, n)
    }
}

data class Rotation2d(val re: Double, val im: Double) {
    fun log() = atan2(im, re)
    fun scaled(k: Double) = exp(log() * k)
    val inverse get() = Rotation2d(re, -im)
    val direction get() = Vector2d(re, im)

    override fun equals(other: Any?): Boolean {
        if(other !is Rotation2d) {
            return false
        }

        return equals(other)
    }

    fun equals(other: Rotation2d) = re.equals(other.re) && im.equals(other.im)

    fun approxEqs(other: Rotation2d, eps: Double = 10e-10) = re.approxEquals(other.re, eps) && im.approxEquals(other.im, eps)

    override fun hashCode(): Int {
        var result = re.hashCode()
        result = 31 * result + im.hashCode()
        return result
    }

    override fun toString(): String {
        return "${Math.toDegrees(log()).rounded()} deg"
    }

    operator fun times(b: Rotation2d) = Rotation2d(this.re * b.re - this.im * b.im, this.re * b.im + this.im * b.re)
    operator fun times(r2: Vector2d) = Vector2d(this.re * r2.x - this.im * r2.y, this.im * r2.x + this.re * r2.y)
    operator fun div(b: Rotation2d) = b.inverse * this
    operator fun plus(incr: Double) = this * exp(incr)

    companion object {
        val zero = exp(0.0)

        fun exp(angleIncr: Double) = Rotation2d(cos(angleIncr), sin(angleIncr))

        fun dir(direction: Vector2d): Rotation2d {
            val dir = direction.normalized()

            return Rotation2d(dir.x, dir.y)
        }
    }
}

fun interpolate(r0: Rotation2d, r1: Rotation2d, t: Double) = Rotation2d.exp(t * (r1 / r0).log()) * r0

data class Rotation2dDual(val re: Dual, val im: Dual) {
    companion object {
        fun exp(angleIncr: Dual) = Rotation2dDual(cos(angleIncr), sin(angleIncr))
        fun const(value: Rotation2d, n: Int = 1) = Rotation2dDual(Dual.const(value.re, n), Dual.const(value.im, n))
        fun const(angleIncr: Double, n: Int = 1) = exp(Dual.const(angleIncr, n))
    }

    val value get() = Rotation2d(re.value, im.value)
    val angularVelocity get() = re * im.tail() - im * re.tail()
    val inverse get() = Rotation2dDual(re, -im)
    val direction get() = Vector2dDual(re, im)

    override fun equals(other: Any?): Boolean {
        if(other !is Rotation2dDual) {
            return false
        }

        return equals(other)
    }

    fun equals(other: Rotation2dDual) = re == other.re && im == other.im

    override fun hashCode(): Int {
        var result = re.hashCode()
        result = 31 * result + im.hashCode()
        return result
    }

    operator fun times(b: Rotation2dDual) = Rotation2dDual(this.re * b.re - this.im * b.im, this.re * b.im + this.im * b.re)
    operator fun times(b: Rotation2d) = Rotation2dDual(this.re * b.re - this.im * b.im, this.re * b.im + this.im * b.re)
    operator fun times(r2: Vector2dDual) = Vector2dDual(this.re * r2.x - this.im * r2.y, this.im * r2.x + this.re * r2.y)
    operator fun times(r2: Vector2d) = Vector2dDual(this.re * r2.x - this.im * r2.y, this.im * r2.x + this.re * r2.y)
}

data class Twist2dIncr(val trIncr: Vector2d, val rotIncr: Double) {
    constructor(xIncr: Double, yIncr: Double, rotIncr: Double) : this(Vector2d(xIncr, yIncr), rotIncr)

    override fun equals(other: Any?): Boolean {
        if(other !is Twist2dIncr) {
            return false
        }

        return equals(other)
    }

    fun equals(other: Twist2dIncr) = trIncr.equals(other.trIncr) && rotIncr.equals(other.rotIncr)

    override fun hashCode(): Int {
        var result = trIncr.hashCode()
        result = 31 * result + rotIncr.hashCode()
        return result
    }

    override fun toString(): String {
        return "$trIncr $rotIncr"
    }
}

data class Twist2dIncrDual(val trIncr: Vector2dDual, val rotIncr: Dual) {
    constructor(xIncr: Dual, yIncr: Dual, rotIncr: Dual) : this(Vector2dDual(xIncr, yIncr), rotIncr)

    val value get() = Twist2dIncr(trIncr.value, rotIncr.value)
    val velocity get() = Twist2dDual(trIncr.tail(), rotIncr.tail())
    fun head(n: Int = 1) = Twist2dIncrDual(trIncr.head(n), rotIncr.head(n))
    fun tail(n: Int = 1) = Twist2dIncrDual(trIncr.tail(n), rotIncr.tail(n))

    override fun equals(other: Any?): Boolean {
        if(other !is Twist2dIncrDual) {
            return false
        }

        return equals(other)
    }

    fun equals(other: Twist2dIncrDual) = trIncr.equals(other.trIncr) && rotIncr.equals(other.rotIncr)

    override fun hashCode(): Int {
        var result = trIncr.hashCode()
        result = 31 * result + rotIncr.hashCode()
        return result
    }

    override fun toString(): String {
        return "$trIncr $rotIncr"
    }

    companion object {
        fun const(trIncr: Vector2d, rotIncr: Double, n: Int = 1) = Twist2dIncrDual(Vector2dDual.const(trIncr, n), Dual.const(rotIncr, n))
    }
}

data class Twist2d(val trVelocity: Vector2d, val rotVelocity: Double) {
    constructor(dx: Double, dy: Double, dTheta: Double) : this(Vector2d(dx, dy), dTheta)

    override fun equals(other: Any?): Boolean {
        if(other !is Twist2d) {
            return false
        }

        return equals(other)
    }

    fun equals(other: Twist2d) = trVelocity.equals(other.trVelocity) && rotVelocity.equals(other.rotVelocity)

    fun approxEqs(other: Twist2d, eps: Double = 10e-10) = trVelocity.approxEqs(other.trVelocity, eps) && rotVelocity.approxEquals(other.rotVelocity, eps)

    override fun toString(): String {
        return "$trVelocity m/s ${Math.toDegrees(rotVelocity).rounded()} deg/s"
    }

    override fun hashCode(): Int {
        var result = trVelocity.hashCode()
        result = 31 * result + rotVelocity.hashCode()
        return result
    }

    operator fun plus(other: Twist2d) = Twist2d(trVelocity + other.trVelocity, rotVelocity + other.rotVelocity)
    operator fun minus(other: Twist2d) = Twist2d(trVelocity - other.trVelocity, rotVelocity - other.rotVelocity)
    operator fun times(scalar: Double) = Twist2d(trVelocity * scalar, rotVelocity * scalar)
    operator fun div(scalar: Double) = Twist2d(trVelocity / scalar, rotVelocity / scalar)

    companion object {
        val zero = Twist2d(Vector2d.zero, 0.0)
    }
}

data class Twist2dDual(val trVelocity: Vector2dDual, val rotVelocity: Dual) {
    constructor(dx: Dual, dy: Dual, dTheta: Dual) : this(Vector2dDual(dx, dy), dTheta)

    val value get() = Twist2d(trVelocity.value, rotVelocity.value)
    fun head(n: Int = 1) = Twist2dDual(trVelocity.head(n), rotVelocity.head(n))
    fun tail(n: Int = 1) = Twist2dDual(trVelocity.tail(n), rotVelocity.tail(n))

    override fun equals(other: Any?): Boolean {
        if(other !is Twist2dDual) {
            return false
        }

        return equals(other)
    }

    fun equals(other: Twist2dDual) = trVelocity.equals(other.trVelocity) && rotVelocity == other.rotVelocity

    override fun toString(): String {
        return "$trVelocity $rotVelocity"
    }

    override fun hashCode(): Int {
        var result = trVelocity.hashCode()
        result = 31 * result + rotVelocity.hashCode()
        return result
    }

    operator fun plus(other: Twist2dDual) = Twist2dDual(trVelocity + other.trVelocity, rotVelocity + other.rotVelocity)
    operator fun minus(other: Twist2dDual) = Twist2dDual(trVelocity - other.trVelocity, rotVelocity - other.rotVelocity)

    companion object {
        fun const(value: Twist2d, n: Int = 1) = Twist2dDual(Vector2dDual.const(value.trVelocity, n), Dual.const(value.rotVelocity, n))
    }
}

data class Pose2d(val translation: Vector2d, val rotation: Rotation2d) {
    constructor(x: Double, y: Double, angle: Double) : this(Vector2d(x, y), Rotation2d.exp(angle))
    constructor(x: Double, y: Double) : this(x, y, 0.0)

    val inverse get() = Pose2d(rotation.inverse * -translation, rotation.inverse)

    fun log(): Twist2dIncr {
        val angle = rotation.log()

        val u2 = 0.5 * angle + snzEps(angle)
        val halfTan = u2 / tan(u2)

        return Twist2dIncr(
            Vector2d(
                halfTan * translation.x + u2 * translation.y,
                -u2 * translation.x + halfTan * translation.y
            ),
            angle
        )
    }

    override fun equals(other: Any?): Boolean {
        if(other !is Pose2d) {
            return false
        }

        return equals(other)
    }

    fun equals(other: Pose2d) = translation.equals(other.translation) && rotation.equals(other.rotation)

    fun approxEqs(other: Pose2d, eps: Double = 10e-10) = translation.approxEqs(other.translation, eps) && rotation.approxEqs(other.rotation, eps)

    override fun hashCode(): Int {
        var result = translation.hashCode()
        result = 31 * result + rotation.hashCode()
        return result
    }

    override fun toString(): String {
        return "$translation $rotation"
    }

    operator fun times(b: Pose2d) = Pose2d(this.translation + this.rotation * b.translation, this.rotation * b.rotation)
    operator fun times(v: Vector2d) = this.translation + this.rotation * v
    operator fun div(b: Pose2d) = b.inverse * this
    operator fun plus(incr: Twist2dIncr) = this * exp(incr)
    operator fun minus(b: Pose2d) = (this / b).log()

    companion object {
        fun exp(incr: Twist2dIncr): Pose2d {
            val rot = Rotation2d.exp(incr.rotIncr)

            val u = incr.rotIncr + snzEps(incr.rotIncr)
            val c = 1.0 - cos(u)
            val s = sin(u)

            return Pose2d(
                Vector2d(
                    (s * incr.trIncr.x - c * incr.trIncr.y) / u,
                    (c * incr.trIncr.x + s * incr.trIncr.y) / u
                ),
                rot
            )
        }
    }
}

data class CurvePose2d(val pose: Pose2d, val curvature: Double)

fun interpolate(a: Pose2d, b: Pose2d, t: Double) = Pose2d(lerp(a.translation, b.translation, t), interpolate(a.rotation, b.rotation, t))

data class Pose2dDual(val translation: Vector2dDual, val rotation: Rotation2dDual) {
    val inverse get() = Pose2dDual(rotation.inverse * -translation, rotation.inverse)
    val value get() = Pose2d(translation.value, rotation.value)
    val velocity get() = Twist2dDual(translation.tail(), rotation.angularVelocity)

    override fun equals(other: Any?): Boolean {
        if(other !is Pose2dDual) {
            return false
        }

        return equals(other)
    }

    fun equals(other: Pose2dDual) = translation.equals(other.translation) && rotation.equals(other.rotation)

    override fun hashCode(): Int {
        var result = translation.hashCode()
        result = 31 * result + rotation.hashCode()
        return result
    }

    override fun toString(): String {
        return "$translation $rotation"
    }

    operator fun times(b: Pose2dDual) = Pose2dDual(this.translation + this.rotation * b.translation, this.rotation * b.rotation)
    operator fun times(b: Pose2d) = Pose2dDual(this.translation + this.rotation * b.translation, this.rotation * b.rotation)
    operator fun div(b: Pose2dDual) = b.inverse * this
    operator fun plus(incr: Twist2dIncr) = this * Pose2d.exp(incr)
}

class VectorKd internal constructor(private val values: DoubleArray) {
    companion object {
        fun ofArray(values: DoubleArray): VectorKd {
            return VectorKd(values.clone())
        }

        fun ofList(values: List<Double>): VectorKd {
            return VectorKd(values.toDoubleArray())
        }

        fun of(vararg values: Double): VectorKd {
            return VectorKd(values.asList().toDoubleArray())
        }
    }

    init {
        require(values.isNotEmpty())
    }

    val size = values.size

    operator fun get(index: Int) = values[index]

    operator fun unaryPlus() = VectorKd(values.map { +it }.toDoubleArray())
    operator fun unaryMinus() = VectorKd(values.map { -it }.toDoubleArray())

    operator fun plus(other: VectorKd): VectorKd {
        validate(this, other)

        val results = DoubleArray(size)

        for (i in 0 until size) {
            results[i] = values[i] + other.values[i]
        }

        return VectorKd(results)
    }

    operator fun minus(other: VectorKd): VectorKd {
        validate(this, other)

        val results = DoubleArray(size)

        for (i in 0 until size) {
            results[i] = values[i] - other.values[i]
        }

        return VectorKd(results)
    }

    operator fun times(scalar: Double): VectorKd {
        val results = DoubleArray(size)

        for (i in 0 until size) {
            results[i] = values[i] * scalar
        }

        return VectorKd(results)
    }

    operator fun div(scalar: Double): VectorKd {
        val results = DoubleArray(size)

        for (i in 0 until size) {
            results[i] = values[i] / scalar
        }

        return VectorKd(results)
    }

    override fun equals(other: Any?): Boolean {
        if (other !is VectorKd) {
            return false
        }

        return values contentEquals other.values
    }

    fun equals(other: VectorKd): Boolean {
        return values contentEquals other.values
    }

    override fun hashCode(): Int {
        return values.contentHashCode()
    }

    override fun toString(): String {
        return values.joinToString(", ")
    }
}

fun kdVectorOf(vararg values: Double): VectorKd {
    return VectorKd(values.asList().toDoubleArray())
}

fun distanceSqr(a: VectorKd, b: VectorKd): Double {
    validate(a, b)

    var result = 0.0

    for (i in 0 until a.size) {
        val d = a[i] - b[i]

        result += d * d
    }

    return result
}

fun distance(a: VectorKd, b: VectorKd) = distanceSqr(a, b).sqrt()

/**
 * Validates that [vector] has the [requiredSize].
 * */
fun validate(vector: VectorKd, requiredSize: Int) {
    require(vector.size == requiredSize)
}

/**
 * Validates that [a] has the same size as [b].
 * */
fun validate(a: VectorKd, b: VectorKd) {
    require(a.size == b.size)
}

/**
 * Validates that the vectors have the same size.
 * */
fun validate(a: VectorKd, b: VectorKd, c: VectorKd, d: VectorKd, e: VectorKd, f: VectorKd) {
    require(a.size == b.size && b.size == c.size && c.size == d.size && d.size == e.size && e.size == f.size)
}

fun Double.toKdVector() : VectorKd {
    return kdVectorOf(this)
}

fun Vector2d.toKdVec(): VectorKd {
    return kdVectorOf(this.x, this.y)
}

fun VectorKd.toVector2d(): Vector2d {
    validate(this, 2)

    return Vector2d(this[0], this[1])
}