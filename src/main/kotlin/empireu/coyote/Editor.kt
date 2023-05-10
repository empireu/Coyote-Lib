package empireu.coyote

import com.google.gson.Gson
import java.io.File
import kotlin.math.absoluteValue
import kotlin.reflect.KClass
import kotlin.reflect.full.isSubtypeOf
import kotlin.reflect.full.primaryConstructor
import kotlin.reflect.full.starProjectedType

/**
 * Coyote editor imports.
 *
 * Do not change [Float] to [Double]. These implementations were designed to produce results equal to the editor implementation.
 * Unfortunately, this was only possible up to a certain point.
 * Changing these to use [Double] would introduce a lot more variance in the results.
 * */

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

// todo replace this when preparing this for use:
fun loadCoyoteProject(): JsonProject =
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


data class BehaviorCreateContext(
    val name: String,
    val runOnce: Boolean,
    val savedData: String,
    val children: Map<Int, List<BehaviorNode>>
) {
    val childNodes get() = children.flatMap { it.value }

    fun validateSubNodes(required: Int) : BehaviorCreateContext {
        require(childNodes.size == required) { "Failed to get required number of sub nodes" }

        return this
    }

    fun validateTerminals(required: Int) : BehaviorCreateContext {
        require(children.size == required) { "Failed to get required number of terminals " }

        return this
    }

    fun one() = this.validateSubNodes(1).childNodes.first()
}

/**
 * Behavior Node Factory. It is used to construct the initial [BehaviorNode]. Further initialization is done using a multi-pass process.
 * @see IBehaviorInitPass
 * */
fun interface IBehaviorFactory<T : BehaviorNode> {
    fun create(context: BehaviorCreateContext): T
}

/**
 * Behavior Node Initialization Pass. All passes are grouped by index (see [BehaviorMapBuilder.add]) and executed when the behavior is being loaded.
 * */
fun interface IBehaviorInitPass<T : BehaviorNode> {
    fun pass(passContext: BehaviorPassContext<T>)
}

data class BehaviorCreateInfo(
    val factory: IBehaviorFactory<BehaviorNode>,
    val passes: List<IBehaviorInitPass<BehaviorNode>>
)

/**
 * Encapsulates information about a specific behavior tree, that belongs to a project.
 * @param root The root node of the tree.
 * @param nodes The nodes in the tree, including [root].
 * */
data class BehaviorTreeInfo(val root: BehaviorNode, val nodes: List<BehaviorNode>)

/**
 * Encapsulates information about all behaviors in a project.
 * @param behaviors The behaviors in the project.
 * */
data class BehaviorProjectInfo(val behaviors: List<BehaviorTreeInfo>)

/**
 * Encapsulates information about a behavior node loading pass.
 * @param node The node being initialized.
 * @param root The root node of [node]'s tree.
 * @param tree The nodes in [node]'s tree.
 * @param project Information about the project [node] belongs to.
 * @param createContext The serialized data of this node.
 * */
data class BehaviorPassContext<T : BehaviorNode>(
    val node: T,
    val root: BehaviorNode,
    val tree: List<BehaviorNode>,
    val project: BehaviorProjectInfo,
    val createContext: BehaviorCreateContext
)

class BehaviorMapBuilder {
    val behaviors = HashMap<String, BehaviorCreateInfo>()

    /**
     * Adds a behavior to this builder.
     * @param name The unique name of this node.
     * @param factory A factory, that will be used to create new instance of the node.
     * @param passes Supplementary initialization pass functions. The passes from all nodes are grouped based on index (the position of the vararg) and then executed, in order to finish initializing the node.
     * */
    inline fun<reified T : BehaviorNode> add(name: String, factory: IBehaviorFactory<T>, vararg passes: IBehaviorInitPass<T>): BehaviorMapBuilder {
        if(behaviors.containsKey(name)) {
            error("Duplicate node $name")
        }

        behaviors[name] = BehaviorCreateInfo(
            { factory.create(it) },
            passes
                .asList()
                .map { callback ->
                    IBehaviorInitPass { ctx ->
                        callback.pass(
                            BehaviorPassContext(
                                ctx.node as T,
                                ctx.root,
                                ctx.tree,
                                ctx.project,
                                ctx.createContext
                            )
                        )
                    }
                }
        )

        return this
    }

    fun build() = BehaviorMap(behaviors.toMap())
}

data class BehaviorMap(val behaviors: Map<String, BehaviorCreateInfo>)

private data class StagedNode(val node: BehaviorNode, val context: BehaviorCreateContext)

/**
 * Constructs the Behavior Tree, starting at [root].
 * @param behaviorMap The factory specification of this Behavior Tree. Only the factory will be invoked to create the node; the passes will not be executed here.
 * @param nodeTracking A mapping of [BehaviorCreateInfo] to a list of the nodes that were created for that specific [BehaviorCreateInfo].
 * */
private fun createNode(root: JsonNode, behaviorMap: BehaviorMap, nodeTracking: HashMap<BehaviorCreateInfo, ArrayList<StagedNode>>) : BehaviorNode {
    val createInfo = behaviorMap.behaviors[root.BehaviorId]
        ?: error("Behavior with ID ${root.BehaviorId} is not registered")

    val context = BehaviorCreateContext(
        root.Name,
        root.ExecuteOnce,
        root.SavedData,
        root.Children.let {
            val map = HashMap<Int, ArrayList<BehaviorNode>>()

            it.forEach { child ->
                val node = createNode(child.Node, behaviorMap, nodeTracking)

                if(map.containsKey(child.TerminalId)) {
                    map[child.TerminalId]!!.add(node)
                }
                else{
                    map[child.TerminalId] = arrayListOf(node)
                }
            }

            return@let map.toMap()
        }
    )

    return createInfo.factory.create(context).also { node ->
        nodeTracking.getOrPut(createInfo) { ArrayList() }.add(StagedNode(node, context))
    }
}

fun loadNodeProject(rootNodes: List<JsonNode>, behaviorMap: BehaviorMap) : BehaviorProjectInfo {
    class TreeInfo(
        val tracking: HashMap<BehaviorCreateInfo, ArrayList<StagedNode>>,
        val root: BehaviorNode,
        val nodes: ArrayList<BehaviorNode>
    )

    val projectTrees = rootNodes.map { jsonRoot ->
        val tracking = HashMap<BehaviorCreateInfo, ArrayList<StagedNode>>()
        val root = createNode(jsonRoot, behaviorMap, tracking)
        val tree = ArrayList<BehaviorNode>()

        fun treeScan(node: BehaviorNode) {
            tree.add(node)

            if(node is IParentBehavior) {
                node.children.forEach { treeScan(it) }
            }
        }

        treeScan(root)

        TreeInfo(
            tracking,
            root,
            tree
        )
    }

    val projectInfo = BehaviorProjectInfo(
        projectTrees.map { behavior ->
            BehaviorTreeInfo(behavior.root, behavior.nodes)
        }
    )

    projectTrees.forEach { treeInfo ->
        class PassInfo(val createInfo: BehaviorCreateInfo, val passCallbacks: ArrayList<IBehaviorInitPass<BehaviorNode>>)

        val passes = HashMap<Int, HashMap<BehaviorCreateInfo, PassInfo>>()

        behaviorMap.behaviors.values.forEach { createInfo ->
            createInfo.passes.forEachIndexed { passIndex, pass ->
                passes
                    .getOrPut(passIndex) { HashMap() }
                    .getOrPut(createInfo) { PassInfo(createInfo, ArrayList()) }
                    .passCallbacks
                    .add(pass)
            }
        }

        passes.keys.sorted().forEach { idx ->
            passes[idx]!!.forEach { (_, passInfo) ->
                val nodes = treeInfo.tracking[passInfo.createInfo]

                if(nodes != null) {
                    passInfo.passCallbacks.forEach { callback ->
                        nodes.forEach { stagedNode ->
                            callback.pass(
                                BehaviorPassContext(
                                    stagedNode.node,
                                    treeInfo.root,
                                    treeInfo.nodes,
                                    projectInfo,
                                    stagedNode.context
                                )
                            )
                        }
                    }
                }
            }
        }
    }

    return projectInfo
}

/**
 * This implementation is small and simple. I did not architect *a system* here.
 * It would have been a lot more difficult to digest by less experienced users that may want to modify it.
 * Also, it is not necessary since we don't plan to extend this.
 * */

interface INamedElement {
    val Name: String
    val Value: Any
}
data class JsonCompositeFlag(override val Name: String, val State: Boolean) : INamedElement{
    override val Value: Any
        get() = State
}
data class JsonCompositeEnum(override val Name: String, val Selected: String) : INamedElement{
    override val Value: Any
        get() = Selected
}
data class JsonCompositeRealValue(override val Name: String, override val Value: Double) : INamedElement
data class JsonCompositeIntegerValue(override val Name: String, override val Value: Int) : INamedElement
data class JsonCompositeTextValue(override val Name: String, override val Value: String) : INamedElement

data class JsonCompositeState(
    val Flags: List<JsonCompositeFlag>,
    val Enums: List<JsonCompositeEnum>,
    val RealSliders: List<JsonCompositeRealValue>,
    val IntegerSliders: List<JsonCompositeIntegerValue>,
    val TextInputFields: List<JsonCompositeTextValue>,
    val RealInputFields: List<JsonCompositeRealValue>,
    val IntegerInputFields: List<JsonCompositeIntegerValue>
) {
    private fun txMap() : Map<String, Any> {
        val result = HashMap<String, Any>()

        fun add(element: INamedElement) {
            if(result.put(element.Name, element.Value) != null){
                error("Duplicate element ${element.Name}")
            }
        }

        (Flags + Enums + RealSliders + IntegerSliders + TextInputFields + RealInputFields + IntegerInputFields).forEach(::add)

        return result
    }

    fun<T : Any> load(k: KClass<T>): T {
        val map = txMap()

        return (k.primaryConstructor ?: error("Failed to get constructor")).let { ctor ->
            ctor.callBy(ctor
                .parameters
                .associateWith { param ->
                    val serializedValue = map[param.name] ?: error("Failed to map ${param.name}")

                    if(param.type.isSubtypeOf(Enum::class.starProjectedType)) {
                        (param.type.classifier as KClass<*>).java.enumConstants.firstOrNull {
                            (it as Enum<*>).name == (serializedValue as String)
                        } ?: error("Failed to parse option $serializedValue in ${param.name}")
                    } else {
                        serializedValue
                    }
                }
            )
        }
    }
}


