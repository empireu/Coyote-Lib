package empireu.coyote

import kotlin.math.PI
import kotlin.math.sqrt
import kotlin.test.Test

class GeometryTests {
    private val Eps = 10e-12

    @Test
    fun testVector2d() {
        assert(Vector2d.zero == Vector2d(0.0, 0.0) && Vector2d.zero == Vector2d(0.0))
        assert(Vector2d.one == Vector2d(1.0, 1.0) && Vector2d.one == Vector2d(1.0))
        assert(Vector2d.unitX == Vector2d(1.0, 0.0))
        assert(Vector2d.unitY == Vector2d(0.0, 1.0))
        assert(Vector2d.unitX + Vector2d.unitY == Vector2d.one)
        assert(Vector2d.zero != Vector2d.one)
        assert(Vector2d.zero.length == 0.0 && Vector2d.zero.lengthSqr == 0.0)
        assert(Vector2d.one.length == sqrt(2.0) && Vector2d.one.lengthSqr == 2.0)
        assert(Vector2d.unitX.length == 1.0 && Vector2d.unitX.lengthSqr == 1.0)
        assert(Vector2d.unitY.length == 1.0 && Vector2d.unitY.lengthSqr == 1.0)
        assert((Vector2d.one * 0.5 + Vector2d.one / 2.0) == Vector2d.one)
        assert(Vector2d.one == Vector2d.one * 2.0 - Vector2d.one)
        assert(Vector2d(1000.0, 1000.0).normalized().length.approxEquals(1.0))
        assert(Vector2d(1000.0, 1000.0).normalized() * sqrt(1000.0 * 1000.0 * 2) == Vector2d(1000.0, 1000.0))
        assert(lerp(Vector2d.zero, Vector2d(1.0, 2.0), 0.0) == Vector2d.zero)
        assert(lerp(Vector2d.zero, Vector2d(1.0, 2.0), 0.5) == Vector2d(1.0, 2.0) / 2.0)
        assert(lerp(Vector2d.zero, Vector2d(1.0, 2.0), 1.0) == Vector2d(1.0, 2.0))
    }

    @Test
    fun testRotation2d() {
        fun areEqual(vararg values: Rotation2d) {
            require(values.size > 1)

            for (i in 1 until values.size) {
                assert(values[i - 1].approxEqs(values[i], Eps))
            }
        }

        val rpi = Rotation2d.exp(PI)

        areEqual(rpi, rpi)
        areEqual(rpi.scaled(1.0), rpi)
        areEqual(rpi.scaled(-1.0), rpi.inverse)
        areEqual(rpi.scaled(0.5), Rotation2d.exp(PI / 2.0))

        areEqual(rpi * rpi, Rotation2d.exp(PI * 2.0), Rotation2d.exp(PI * 4.0), Rotation2d.zero)
        areEqual(rpi * Rotation2d.exp(-PI), Rotation2d.zero)
        areEqual(rpi * rpi.inverse, Rotation2d.zero)

        assert((rpi * Vector2d.unitX).approxEqs(-Vector2d.unitX, Eps))
        assert((Rotation2d.exp(PI * 2.0) * Vector2d.unitX).approxEqs(Vector2d.unitX, Eps))
        assert((Rotation2d.exp(PI * 8.0) * Vector2d.unitX).approxEqs(Vector2d.unitX, Eps))

        areEqual(interpolate(Rotation2d.zero, rpi, 0.0), Rotation2d.zero)
        areEqual(interpolate(Rotation2d.zero, rpi, 1.0), rpi)
        areEqual(interpolate(Rotation2d.zero, rpi, 0.5), Rotation2d.exp(PI / 2.0))
        areEqual(interpolate(Rotation2d.zero, rpi, 0.25), Rotation2d.exp(PI / 4.0))

        rangeScan(start = 0.0, end = 1.0) { t ->
            areEqual(interpolate(rpi, rpi, t), rpi)
        }

        rangeScanRec({ vec ->
            val a = Rotation2d.exp(vec[0])
            val b = Rotation2d.exp(vec[1])

            areEqual(a * (b / a), b)
        }, start = -100.0, end = 100.0, steps = 1000, layers = 2)
    }

    @Test
    fun testPose2d() {
        fun areEqual(vararg values: Pose2d) {
            require(values.size > 1)

            for (i in 1 until values.size) {
                assert(values[i - 1].approxEqs(values[i], Eps))
            }
        }

        rangeScanRec({ vec ->
            val a = Pose2d(vec[0], vec[1], vec[2])
            val b = Pose2d(vec[3], vec[4], vec[5])

            areEqual(a + (b - a), b)
            areEqual(a * (b / a), b)
        }, start = -100.0, end = 100.0, steps = 10, layers = 6)
    }
}
