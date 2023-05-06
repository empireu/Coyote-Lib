package empireu.coyote

import com.google.gson.Gson
import java.io.File
import kotlin.math.absoluteValue

data class JsonVec2(val X: Float, val Y: Float)

data class JsonTranslationPoint(private val Position: JsonVec2, private val Velocity: JsonVec2, private val Acceleration: JsonVec2) {
    val position get() = kdVectorOf(Position.X.toDouble(), Position.Y.toDouble())
    fun velocity(scale: Float) = kdVectorOf(((Velocity.X - Position.X) * scale).toDouble(), ((Velocity.Y - Position.Y) * scale).toDouble())
    fun acceleration(scale: Float) = kdVectorOf(((Acceleration.X - Position.X) * scale).toDouble(), ((Acceleration.Y - Position.Y) * scale).toDouble())
}

data class JsonRotationPoint(private val Position: JsonVec2, private val Heading: JsonVec2, val Parameter: Double) {
    val heading get() = Vector2d((Heading.X - Position.X).toDouble(), (Heading.Y - Position.Y).toDouble())
}

data class JsonMotionConstraints(
    val LinearVelocity: Double,
    val LinearAcceleration: Double,
    val LinearDeacceleration: Double,
    val AngularVelocity: Double,
    val AngularAcceleration: Double,
    val CentripetalAcceleration: Double
)

data class JsonGenerationParameters(
    val Dx: Double,
    val Dy: Double,
    val DAngleTranslation: Double,
    val DParameterTranslation: Double,
    val DAngleRotation: Double,
    val DParameterRotation: Double
)

data class JsonMarker(
    val Position: JsonVec2,
    val Parameter: Double,
    val Name: String
)

data class JsonChild(
    val TerminalId: Int,
    val Node: JsonNode
)

data class JsonNode(
    val Position: JsonVec2,
    val BehaviorId: String,
    val Name: String,
    val Description: String,
    val ExecuteOnce: Boolean,
    val SavedData: String,
    val Children: List<JsonChild>
)

data class JsonMotionProject(
    val TranslationPoints: List<JsonTranslationPoint>,
    val RotationPoints: List<JsonRotationPoint>,
    val Markers: List<JsonMarker>,
    val Scale: Float,
    val Constraints: JsonMotionConstraints,
    val Parameters: JsonGenerationParameters,
    val Version: Int
) {
    init {
        if(TranslationPoints.size < 2) {
            error("Invalid motion project. At least 2 translation points are needed.")
        }

        if(RotationPoints.size == 1) {
            error("Invalid motion project. The rotation spline is incomplete.")
        }
    }

    fun createTranslationSpline(): QuinticSpline {
        val segments = ArrayList<QuinticSplineSegment>()

        for(i in 1 until TranslationPoints.size) {
            val p0 = TranslationPoints[i - 1]
            val p1 = TranslationPoints[i]

            segments.add(
                QuinticSplineSegment(
                    p0.position,
                    p0.velocity(Scale),
                    p0.acceleration(Scale),
                    p1.acceleration(Scale),
                    p1.velocity(Scale),
                    p1.position
                )
            )
        }

        return QuinticSpline(segments)
    }

    val hasRotationSpline get() = RotationPoints.isNotEmpty()

    fun createRotationSpline(): QuinticSplineMapped {
        require(hasRotationSpline)

        val builder = QuinticSplineMappedBuilder(1)

        var angle = 0.0

        for(i in RotationPoints.indices) {
            val rotationPoint = RotationPoints[i]
            val currentRotation = Rotation2d.dir(rotationPoint.heading)

            if(i == 0) {
                angle = currentRotation.log()
            }
            else{
                val previousRotationPoint = RotationPoints[i - 1]

                if(rotationPoint.Parameter.approxEquals(previousRotationPoint.Parameter)) {
                    continue
                }

                angle += (currentRotation / Rotation2d.dir(previousRotationPoint.heading)).log()
            }

            builder.add(rotationPoint.Parameter, displacement = kdVectorOf(angle), kdVectorOf(0.0), kdVectorOf(0.0))
        }

        return builder.build()
    }
}

data class JsonNodeProject(val RootNodes: List<JsonNode>)

data class JsonProject(
    val MotionProjects: Map<String, JsonMotionProject>,
    val NodeProjects: Map<String, JsonNodeProject>
)

fun loadCoyoteProject() =
    Gson().fromJson(
        File(System.getenv("COYOTE_PROJECTS") + "/proj.awoo").readText(),
        JsonProject::class.java
    )

data class TrajectoryMarker(
    val name: String,
    val point: TrajectoryPoint
)

data class EditorTrajectory(
    val trajectory: Trajectory,
    val markers: List<TrajectoryMarker>
)

fun loadTrajectory(motionProject: JsonMotionProject): EditorTrajectory {
    val pathPoints = ArrayList<CurvePoseParam<Percentage>>()

    val rotationSpline =
        if(motionProject.hasRotationSpline) motionProject.createRotationSpline()
        else null

    val translationSpline = motionProject.createTranslationSpline()

    translationSpline.getPointsWpi(
        pathPoints,
        t0 = 0.0,
        t1 = 1.0,
        admissibleT = motionProject.Parameters.DParameterTranslation,
        admissibleIncr = Twist2dIncr(
            motionProject.Parameters.Dx,
            motionProject.Parameters.Dy,
            motionProject.Parameters.DAngleTranslation),
        maxIterations = Int.MAX_VALUE
    ) { t0, t1 ->
        if ((t0 - t1).absoluteValue > motionProject.Parameters.DParameterRotation) {
            return@getPointsWpi true
        }

        if (rotationSpline == null) {
            return@getPointsWpi false
        }

        val r0 = rotationSpline.evaluatePosition(t0)
        val r1 = rotationSpline.evaluatePosition(t1)

        return@getPointsWpi (r0[0] - r1[0]).absoluteValue > motionProject.Parameters.DAngleRotation
    }

    // Copy the path specific poses into curve poses, and replace tangent headings with spline headings (if we have a rotation spline)
    val trajectoryPath = ArrayList<CurvePose2d>(pathPoints.size)

    if(rotationSpline != null){
        for (i in 0 until pathPoints.size) {
            val point = pathPoints[i]

            trajectoryPath.add(
                CurvePose2d(
                    Pose2d(
                        point.curvePose.pose.translation,
                        Rotation2d.exp(
                            rotationSpline.evaluatePosition(point.param)[0]
                        )
                    ),
                    point.curvePose.curvature
                )
            )
        }
    } else {
        for (i in 0 until pathPoints.size) {
            trajectoryPath.add(pathPoints[i].curvePose)
        }
    }

    val constraints = BaseTrajectoryConstraints(
        linearVelocity = motionProject.Constraints.LinearVelocity,
        linearAcceleration = motionProject.Constraints.LinearAcceleration,
        linearDeacceleration = motionProject.Constraints.LinearDeacceleration,
        angularVelocity = motionProject.Constraints.AngularVelocity,
        angularAcceleration = motionProject.Constraints.AngularAcceleration,
        centripetalAcceleration = motionProject.Constraints.CentripetalAcceleration
    )

    val profile = TrajectoryGenerator.generateProfile(trajectoryPath, constraints)

    // Now, scan the profile and map the trajectory points to markers.

    val pendingMarkers = ArrayDeque<JsonMarker>().also { pending ->
        motionProject.Markers.forEach { marker ->
            pending.addLast(marker)
        }
    }

    val trajectoryMarkers = ArrayList<TrajectoryMarker>()

    for (i in profile.indices){
        if(pendingMarkers.isEmpty()){
            break
        }

        val pathParam = pathPoints[i].param
        val pending = pendingMarkers.first()

        if(pathParam >= pending.Parameter){
            pendingMarkers.removeFirst()

            trajectoryMarkers.add(TrajectoryMarker(pending.Name, profile[i]))
        }
    }

    if(pendingMarkers.isNotEmpty()) {
        error("Could not map all markers $pendingMarkers")
    }

    return EditorTrajectory(
        Trajectory(profile),
        trajectoryMarkers
    )
}



