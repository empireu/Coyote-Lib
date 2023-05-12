package empireu.coyote

import kotlin.math.*

/**
 * Maps [v] from a source range to a destination range.
 * @param srcMin The minimum value in [v]'s range.
 * @param srcMax The maximum value in [v]'s range.
 * @param dstMin The resulting range minimum.
 * @param dstMax The resulting range maximum.
 * @return [v] mapped from the source range to the destination range.
 * */
fun map(v: Double, srcMin: Double, srcMax: Double, dstMin: Double, dstMax: Double): Double {
    return dstMin + (v - srcMin) * (dstMax - dstMin) / (srcMax - srcMin)
}

fun Double.mappedTo(srcMin: Double, srcMax: Double, dstMin: Double, dstMax: Double): Double = map(this, srcMin, srcMax, dstMin, dstMax)

/**
 * Linearly interpolates [a] to [b] using the 0-1 parameter [factor].
 * **The [factor] is not clamped implicitly!**
 * */
fun lerp(a: Double, b: Double, factor: Double): Double = (1.0 - factor) * a + factor * b
fun approxEqual(a: Double, b: Double, epsilon: Double = 10e-6): Boolean = abs(a - b) < epsilon
fun Double.approxEquals(other: Double, epsilon: Double = 10e-6): Boolean = approxEqual(this, other, epsilon)
fun Double.sqrt() = sqrt(this)

infix fun Double.approxEquals(other: Double): Boolean = approxEqual(this, other, 10e-6)

/**
 * Returns the max of [a] and [b].
 * If both values are [Double.NaN], an error is produced.
 * If [a] is [Double.NaN], [b] is returned.
 * If [b] is [Double.NaN], [a] is returned.
 * */
fun maxNaN(a: Double, b: Double): Double {
    if (a.isNaN() && b.isNaN()) {
        error("Both a and b were NaN")
    }

    if (a.isNaN()) {
        return b
    }

    if (b.isNaN()) {
        return a
    }

    return max(a, b)
}

/**
 * Returns the min of [a] and [b].
 * If both values are [Double.NaN], an error is produced.
 * If [a] is [Double.NaN], [b] is returned.
 * If [b] is [Double.NaN], [a] is returned.
 * */
fun minNaN(a: Double, b: Double): Double {
    if (a.isNaN() && b.isNaN()) {
        error("Both a and b were NaN")
    }

    if (a.isNaN()) {
        return b
    }

    if (b.isNaN()) {
        return a
    }

    return min(a, b)
}

/**
 * Throws an error if [this] is [Double.NaN]
 * */
fun Double.requireNotNaN(): Double {
    require(!this.isNaN()) { "Value was NaN" }

    return this
}

/**
 * Returns the square of this number.
 * */
fun Double.sqr(): Double = this * this

/**
 * Returns [this] with the specified [sign]. NaN is not permitted in either [this] or [sign].
 * */
fun Double.signed(sign: Double): Double {
    // Kotlin sign function docs:
    /**
     * Returns the first floating-point argument with the sign of the second floating-point argument.
     * Note that unlike the StrictMath.copySign method,
     * this method does not require NaN sign arguments to be treated as positive values;
     * implementations are permitted to treat some NaN arguments as positive and other NaN arguments as negative to allow greater performance.
     * */

    // I added NaN handling here (with requireNotNaN, which will throw if the sign or this is NaN)

    this.requireNotNaN()
    sign.requireNotNaN()

    if (this == 0.0 || sign == 0.0) {
        return 0.0
    }

    return if (sign(this) != sign(sign))
        -this
    else this
}

/**
 * Rounds the number to the specified number of [decimals].
 * */
fun Double.rounded(decimals: Int = 3): Double {
    var multiplier = 1.0
    repeat(decimals) { multiplier *= 10 }
    return round(this * multiplier) / multiplier
}

fun Double.minWith(other: Double) = min(this, other)
fun Double.maxWith(other: Double) = max(this, other)

data class Range(val min: Double, val max: Double) {
    // maybe allow infinite here?

    companion object {
        val R = Range(Double.MIN_VALUE, Double.MAX_VALUE)
    }

    override fun toString(): String {
        return "$min:$max"
    }

    val isValid get() = min.isFinite() && max.isFinite() && min < max
}

/**
 * Intersects two numeric ranges.
 * @return The intersection range. If the range is valid, as per [Range.isValid], an intersection does exist between [r1] and [r2]. If not, then [r1] and [r2] do not intersect.
 * */
fun intersect(r1: Range, r2: Range): Range = Range(max(r1.min, r2.min), min(r1.max, r2.max))

/**
 * [Dual Number](https://en.wikipedia.org/wiki/Dual_number) auto-differentiation.
 * The algorithm is inspired by [Higher Order Automatic Differentiation with Dual Numbers](https://pp.bme.hu/eecs/article/view/16341).
 * */
class Dual private constructor(private val values: DoubleArray) {
    /**
     * Constructs a [Dual] from the value [x] and the [tail].
     * */
    constructor(x: Double, tail: Dual): this(
        DoubleArray(tail.values.size + 1).also {
            it[0] = x

            for (i in 0 until tail.values.size) {
                it[i + 1] = tail.values[i]
            }
        }
    )

    operator fun get(index: Int) = values[index]

    val size get() = values.size
    val isReal get() = values.size == 1

    /**
     * Gets the first value in this [Dual].
     * */
    val value get() = values[0]

    /**
     * Gets the values at the start of the [Dual], ignoring the last [n] values.
     * */
    fun head(n: Int = 1) = Dual(DoubleArray(size - n) { values[it] })

    /**
     * Gets the values at the end of the [Dual], ignoring the first [n] values.
     * */
    fun tail(n: Int = 1) = Dual(DoubleArray(size - n) { values[it + n] })

    operator fun unaryPlus(): Dual {
        return this
    }

    operator fun unaryMinus() = Dual(
        DoubleArray(size).also {
            for (i in it.indices){
                it[i] = -this[i]
            }
        }
    )

    operator fun plus(other: Dual): Dual =
        if (this.isReal || other.isReal) const(this[0] + other[0])
        else Dual(this.value + other.value, this.tail() + other.tail())

    operator fun minus(other: Dual): Dual =
        if (this.isReal || other.isReal) const(this[0] - other[0])
        else Dual(this.value - other.value, this.tail() - other.tail())

    operator fun times(other: Dual): Dual =
        if (this.isReal || other.isReal) const(this[0] * other[0])
        else Dual(this.value * other.value, this.tail() * other.head() + this.head() * other.tail())

    operator fun div(other: Dual): Dual =
        if (this.isReal || other.isReal) const(this[0] / other[0])
        else Dual(this.value / other.value, (this.tail() * other - this * other.tail()) / (other * other))

    inline fun function(x: ((Double) -> Double), dxFront: ((Dual) -> Dual)): Dual =
        if (this.isReal) const(x(this.value))
        else Dual(x(this.value), dxFront(this.head()) * this.tail())

    operator fun plus(const: Double) = Dual(values.clone().also { it[0] += const })
    operator fun minus(const: Double) = Dual(values.clone().also { it[0] -= const })

    private inline fun mapValues(transform: ((Double) -> Double)) = Dual(DoubleArray(values.size) { i -> transform(values[i])})
    operator fun times(constant: Double) = mapValues { v -> v * constant }
    operator fun div(constant: Double) = mapValues { v -> v / constant }

    override fun equals(other: Any?): Boolean {
        if (this === other){
            return true
        }

        if (javaClass != other?.javaClass) {
            return false
        }

        other as Dual

        if (!values.contentEquals(other.values)){
            return false
        }

        return true
    }

    override fun hashCode(): Int {
        return values.contentHashCode()
    }

    override fun toString(): String {
        if(values.isEmpty()) {
            return "empty"
        }

        return values
            .mapIndexed { i, v -> "x$i=$v" }
            .joinToString(", ")
    }

    companion object {
        fun const(x: Double, n: Int = 1) = Dual(DoubleArray(n).also { it[0] = x })

        fun variable(v: Double, n: Int = 1) = Dual(
            DoubleArray(n).also {
                it[0] = v;
                if(n > 1){
                    it[1] = 1.0
                }
            }
        )

        fun of(vararg values: Double) = Dual(values.asList().toDoubleArray())
    }
}

operator fun Double.plus(dual: Dual) = Dual.const(this, dual.size) + dual
operator fun Double.minus(dual: Dual) = Dual.const(this, dual.size) - dual
operator fun Double.times(dual: Dual) = Dual.const(this, dual.size) * dual
operator fun Double.div(dual: Dual) = Dual.const(this, dual.size) / dual

fun sin(d: Dual): Dual = d.function({ sin(it) }) { cos(it) }
fun cos(d: Dual): Dual = d.function({ cos(it) }) { -sin(it) }
fun pow(d: Dual, n: Double): Dual = d.function({ it.pow(n) }) { n * pow(it, n - 1) }
fun sqrt(d: Dual): Dual = d.function({ sqrt(it) }) { (Dual.const(1.0, d.size) / (Dual.const(2.0, d.size) * sqrt(it))) }
fun Dual.sqr() = this * this

/**
 * Epsilon Sign-Non-Zero function from the [SymForce](https://arxiv.org/abs/2204.07889) paper.
 * */
fun snzEps(a: Double): Double {
    if (a >= 0.0)
    {
        return 2.2e-15;
    }

    return -2.2e-15;
}

fun Double.nonZero() = this + snzEps(this)