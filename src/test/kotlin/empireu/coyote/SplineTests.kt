package empireu.coyote

import kotlin.test.Test

class SplineTests {
    @Test
    fun quinticSplineGetPointsFnvScan() {
        val stream = FnvStream()

        val spline = QuinticSpline(
            listOf(
                QuinticSplineSegment(
                    kdVectorOf(0.0, 1.0),
                    kdVectorOf(1.0, 0.0),
                    kdVectorOf(2.0, 1.0),
                    kdVectorOf(3.0, 0.0),
                    kdVectorOf(4.0, 1.0),
                    kdVectorOf(5.0, 0.0)
                ),
                QuinticSplineSegment(
                    kdVectorOf(5.0, 0.0),
                    kdVectorOf(4.0, 1.0),
                    kdVectorOf(3.0, 1.0),
                    kdVectorOf(2.0, 1.0),
                    kdVectorOf(1.0, 1.0),
                    kdVectorOf(0.0, 1.0)
                )
            )
        )

        val points = ArrayList<CurvePoseParam<Percentage>>()

        spline.getPointsWpi(
            points,
            0.0,
            1.0,
            0.01,
            Twist2dIncr(0.0001, 0.0001, 0.0001)
        )

        points.map { it.curvePose }.hashScan(stream)

        assert(stream.result == 4368584110474596548)
    }
}