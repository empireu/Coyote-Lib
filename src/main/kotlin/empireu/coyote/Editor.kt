package empireu.coyote

class FnvStream {
    var result: Long = basisOffset
        private set

    fun add(b: UByte) {
        result = result xor b.toLong()
        result *= prime
    }

    fun add(long: Long) {
        add((long shr 0 and 0xFF).toUByte())
        add((long shr 8 and 0xFF).toUByte())
        add((long shr 16 and 0xFF).toUByte())
        add((long shr 24 and 0xFF).toUByte())
        add((long shr 32 and 0xFF).toUByte())
        add((long shr 40 and 0xFF).toUByte())
        add((long shr 48 and 0xFF).toUByte())
        add((long shr 56 and 0xFF).toUByte())
    }

    fun add(double: Double) {
        add(java.lang.Double.doubleToRawLongBits(double))
    }

    fun add(vector: Vector2d) {
        add(vector.x)
        add(vector.y)
    }

    fun add(pose: Pose2d) {
        add(pose.translation)
        add(pose.rotation.direction)
    }

    companion object {
        private val basisOffset: Long = java.lang.Long.parseUnsignedLong("14695981039346656037")
        private const val prime: Long = 1099511628211
    }
}

fun List<CurvePose2d>.hashScan(stream: FnvStream) {
    this.forEach { cp ->
        stream.add(cp.pose)
        stream.add(cp.curvature)
    }
}