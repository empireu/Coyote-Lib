package empireu.coyote

import kotlin.test.Test

class FnvTests {
    // Tested against the editor implementation:
    @Test
    fun poseTest() {
        val stream = FnvStream()

        stream.add(0.0)
        stream.add(1.0)
        stream.add(Vector2d(100.0, -100.0))
        stream.add(Pose2d(10.0, 20.0, 30.0))

        assert(stream.result == 3744250648927715553)
    }
}