package empireu.coyote

import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.test.Test
import kotlin.test.assertEquals

class DualTests {
    private val EPS = 10e-12

    private fun rangeScanDual(derivatives: Int = 3, start: Double = 0.0, end: Double = 10.0, steps: Int = 10000, action: ((Double, Dual) -> Unit)) {
        rangeScan(start = start, end = end, steps = steps) { x ->
            action(x, Dual.variable(x, derivatives + 1))
        }
    }

    private fun areEqual(a: Double, b: Double) {
        assertEquals(a, b, EPS)
    }

    @Test
    fun sqrtTest() {
        rangeScanDual(start = 1.0) { x, xDual ->
            val v = sqrt(xDual)

            areEqual(v.value, sqrt(x))
            areEqual(v[1], 1.0 / (2.0 * sqrt(x)))
            areEqual(v[2], -1.0 / (4.0 * x.pow(3.0 / 2.0)))
            areEqual(v[3], 3.0 / (8.0 * x.pow(5.0 / 2.0)))
        }
    }

    @Test
    fun sinTest() {
        rangeScanDual { x, xDual ->
            val v = sin(xDual)

            areEqual(v.value, sin(x))
            areEqual(v[1], cos(x))
            areEqual(v[2], -sin(x))
            areEqual(v[3], -cos(x))
        }
    }

    @Test
    fun cosTest() {
        rangeScanDual { x, xDual ->
            val v = cos(xDual)

            areEqual(v.value, cos(x))
            areEqual(v[1], -sin(x))
            areEqual(v[2], -cos(x))
            areEqual(v[3], sin(x))
        }
    }

    @Test
    fun powTest() {
        rangeScan(start = 1.0, end = 4.0, steps = 100) { power ->
            rangeScanDual(start = 1.0, steps = 1000) { x, xDual ->
                val v = pow(xDual, power)

                areEqual(v.value, x.pow(power))
                areEqual(v[1], power * x.pow(power - 1))
                areEqual(v[2], (power - 1.0) * power * x.pow(power - 2))
                areEqual(v[3], (power - 2.0) * (power - 1.0) * power * x.pow(power - 3))
            }
        }
    }
/*
    @Test
    fun hermiteTest() {
        fun hermiteQuintic(p0: Double, v0: Double, a0: Double, a1: Double, v1: Double, p1: Double, t: Dual): Dual {
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

        rangeScanRec({ vector ->
            rangeScanDual(start = 0.0, end = 1.0, derivatives = 2, steps = 10) { t, tDual ->
                val p0 = vector[0]
                val v0 = vector[1]
                val a0 = vector[2]
                val a1 = vector[3]
                val v1 = vector[4]
                val p1 = vector[5]

                val hermite = hermiteQuintic(p0, v0, a0, a1, v1, p1, tDual)
            }
        }, start = -100.0, end = 100.0, steps = 5, layers = 6)
    }*/
}