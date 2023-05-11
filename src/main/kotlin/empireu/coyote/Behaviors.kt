package empireu.coyote

import com.google.gson.Gson
import com.google.gson.annotations.SerializedName
import kotlin.reflect.KClass

/**
 * [Behavior Tree Implementation](https://en.wikipedia.org/wiki/Behavior_tree_(artificial_intelligence,_robotics_and_control))
 *
 * Conventions:
 * - Proxy nodes with empty sub-node sets must result in [BehaviorStatus.Success].
 * - Decorator nodes must have the one child. Not having it **is an error**.
 * - Nodes must not store any _Execution State_ in their class. It must be stored in the *context*. Storing _Constant_ state (e.g. storing a trajectory) is allowed.
 * - Nodes should not have any expensive initialization logic in the *update* path, since it would slow down execution when external calls and repeat nodes (or any node that creates extra context trees) are concerned.
 *
 * Addendum - **Behavior Context**
 *
 *  I chose to store states in special storage objects because, counter-intuitively, it makes state management easier.
 *  *Case 1: External Call*
 *
 *  Reading the documentation on [BehaviorCallNode] is a prerequisite.
 *
 *  It is necessary to understand what state needs to be stored by the behavior tree:
 *  1. Immutable, runtime data.
 *      - This kind of data is created (synthesized) by the node once, at startup. Its state does not change during the execution of the tree, and it can be used safely in multiple *concurrent paths*.
 *      - Examples:
 *          - Trajectories (the motion node).
 *
 *  2. Status
 *      - This is the current execution state of the node, represented by [BehaviorStatus]. Most nodes (except those with [BehaviorNode.runOnce] set to false) will need this to control the execution of the behavior.
 *      - It changes during execution
 *      - Is unique to every node
 *      - It has special storage ([BehaviorContext.status])
 *
 *  3. Process (execution) state
 *      - Data needed by the algorithms implemented in the node
 *      - It changes during execution
 *      - Is unique to every node
 *      - Examples:
 *          - The controller state of an actuator
 *
 *  Storing **2** and **3** in the actual node instance would need special strategies to be employed by call nodes and repeater nodes:
 *  - Call nodes re-creating the entire tree, starting from the caller
 *      - This is not an issue, and is a sensible approach. It is akin to "splicing" the tree.
 *
 *  - Repeater nodes implementing one of the following two strategies:
 *      - Special clean-up routines (resetting the state of the entire downstream, after an iteration is finished)
 *          - This is not ideal, because it introduces more complexity when dealing with concurrent paths
 *      - Re-creating the entire tree every full iteration
 *          - This is also not ideal because it may be a bit costly to do so (re-synthesizing data described in **1**). This can be mitigated with caching mechanisms (such as storing the data in a static context).
 *
 *  The [BehaviorContext] is a better approach that addresses the issues presented above.
 *      - It does not introduce a large performance penalty
 *      - Generalizes to all node behaviors used so far, so no special logic is needed in the base node
 *      - Concurrent paths can be handled without issues by creating separate contexts, and making the behavior nodes use the [BehaviorContext] for **all execution state storage**.
 *
 *  The only downside is that is requires some boilerplate code (storage keys to access [BehaviorContext.storage]).
 *  Call nodes and repeater nodes are implemented using the following strategies:
 *      - Call context trees
 *          - "Delegate" (call or repeater nodes) create a special context that gets stored in the **parent context** (the context passed down to their `getStatus` method
 *          - The special context is passed down to the `getStatus` method of the delegated nodes
 *      - State storage
 *          - Process state is stored in [BehaviorContext.storage].
 *          - The *Delegate Node* strategy will result in one stored state per behavior path (concurrent path) or iteration.
 * */

enum class BehaviorStatus {
    Success,
    Failure,
    Running;

    val inverse: BehaviorStatus get() = when(this) {
        Success -> Failure
        Failure -> Success
        Running -> error("Cannot get inverse of $Running")
    }

    companion object {
        /**
         * Similar to the logical operator AND.
         * - If any of the two is [Running], the result is [Running].
         * - If any of the two is [Failure], the result is [Failure].
         * - If both are [Success], the result is [Success].
         * */
        fun and(a: BehaviorStatus, b: BehaviorStatus): BehaviorStatus {
            if(a == Running || b == Running) {
                return Running
            }

            if(a == Failure || b == Failure) {
                return Failure
            }

            require(a == Success && b == Success)

            return Success
        }
    }
}

/**
 * Behavior Context for state storage.
 * All Behavior Tree state is stored in a context.
 * */
class BehaviorContext {
    /**
     * State storage for [BehaviorStatus] of running nodes (used by Run Once).
     * */
    val status = HashMap<BehaviorNode, BehaviorStatus>()

    /**
     * Custom storage for nodes.
     * A custom key should be implemented to avoid any collisions between nodes.
     * */
    val storage = HashMap<Any, Any>()

    /**
     * Gets a value from [storage]. If it does not exist, [valueFactory] is invoked, and the result is stored and returned.
     * This is equivalent to **computeIfAbsent**, but wraps the casts for convenience.
     * */
    inline fun<TKey, reified TValue> getOrStore(key: TKey, crossinline valueFactory: ((TKey) -> TValue)): TValue {
        val k = key as Any

        if(!storage.containsKey(k)){
            storage[k] = valueFactory(key) as Any
        }

        if(storage[k] !is TValue) {
            error("Invalid data ${storage[k]} stored at $key")
        }

        return storage[k] as TValue
    }

    /**
     * Gets a value from [storage].
     * */
    inline fun<TKey, reified TValue> getStored(key: TKey): TValue {
        return storage[key as Any] as? TValue ?: error("Storage didn't have key $key")
    }
}

/**
 * Implemented by nodes that have children nodes.
 * Used to scan the tree
 * */
interface IParentBehavior {
    val children: List<BehaviorNode>
}

/**
 * @param name Display name of this node.
 * @param runOnce
 * If true, the [update] logic will only be executed until a non-running result is obtained. If [BehaviorStatus.Success] or [BehaviorStatus.Failure] is returned, this state will be stored in [BehaviorContext.status] and will be returned every time [getStatus] is called.
 * Condition nodes may set [runOnce] to false.
 * This will ensure that the [update] logic is always invoked when [getStatus] is called.
 * Then, [BehaviorStatus.Success] and [BehaviorStatus.Failure] can be used to control and condition the execution of sibling nodes, based on the behavior of the parent node.
 * */
abstract class BehaviorNode(val name: String, val runOnce: Boolean) {
    open fun getStatus(context: BehaviorContext): BehaviorStatus {
        var lastStatus = context.status[this]

        if(!runOnce || // If the node is not runOnce, we don't care about the previous status.
            lastStatus == null || // This is the first time getStatus is called on this node (of course, down the current path described by "context")
            lastStatus == BehaviorStatus.Running // This node has not completed running. It was started in some previous iteration, and needs more updates to complete.
        ) {
            lastStatus = update(context).also {
                context.status[this] = it
            }
        }

        return lastStatus
    }

    protected abstract fun update(context: BehaviorContext): BehaviorStatus
}

/**
 * Behavior node with an arbitrary number of [children] nodes.
 * By convention, [children] can also be empty.
 * */
abstract class BehaviorProxyNode(name: String, runOnce: Boolean, override val children: List<BehaviorNode>) : BehaviorNode(name, runOnce), IParentBehavior

/**
 * Behavior node with a single child node.
 * */
abstract class BehaviorDecoratorNode(name: String, runOnce: Boolean, val child: BehaviorNode) : BehaviorNode(name, runOnce), IParentBehavior {
    override val children: List<BehaviorNode>
        get() = listOf(child)

    final override fun update(context: BehaviorContext): BehaviorStatus {
        return transform(
            child.getStatus(context)
        )
    }

    /**
     * Called to transform the actual [status] into the required [status].
     * [BehaviorStatus.Running] *should not be transformed*, but that is not enforced.
     * */
    protected abstract fun transform(status: BehaviorStatus): BehaviorStatus
}

/**
 * Sequence Node Implementation.
 * - Child nodes will be updated in order.
 * - When Running is signaled by a node, the execution stops and the node signals Running.
 * - When Success is signaled by a node, the execution moves onto the next node *in sequence*.
 * - When Failure is signaled by a node, the Sequence Node will yield the final status Failure.
 * If all nodes were executed successfully, Success will be signaled by the Sequence Node.
 * */
class BehaviorSequenceNode(name: String, runOnce: Boolean, children: List<BehaviorNode>) : BehaviorProxyNode(name, runOnce, children) {
    override fun update(context: BehaviorContext): BehaviorStatus {
        children.forEach {
            val status = it.getStatus(context)

            if(status != BehaviorStatus.Success) {
                // It is either Failure or Running.
                // If it is Failure and "runOnce" is set to true, this effectively becomes the final result of the node.

                return status
            }
        }

        // Also returns Success for empty subtree.

        return BehaviorStatus.Success
    }
}

/**
 * Parallel Node Implementation.
 * - Child nodes will be updated in order.
 * - Child behaviors will run at the same time (the Running status is "ignored", and `getStatus` will be called on all child nodes in a single call to the Parallel Node's `getStatus`)
 * - When Failure is signaled by a node, Failure will be signaled by the Parallel Node.
 * This behavior is similar to that of the Sequence Node.
 * If all nodes were executed successfully, Success will be signaled by the Parallel Node.
 * The Parallel Node does not perform any validation on the subtrees; it is assumed that the editor (which has various connection rules) was used to create the behavior.
 *
 * To give an example of why validation is necessary, consider any node that controls a specific actuator. Updating such nodes in parallel is inherently erroneous (the physical device cannot complete two actions at the same time).
 * */
class BehaviorParallelNode(name: String, runOnce: Boolean, children: List<BehaviorNode>) : BehaviorProxyNode(name, runOnce, children) {
    override fun update(context: BehaviorContext): BehaviorStatus {
        var finished = true  // If we don't have any children, "finished" won't be mutated.
        // We will return Success when this flag is true.

        children.forEach {
            val status = it.getStatus(context)

            if(status == BehaviorStatus.Failure) {
                // If any of the nodes signal Failure, we will also signal failure.

                return BehaviorStatus.Failure
            }

            if(status != BehaviorStatus.Success) {
                require(status == BehaviorStatus.Running)

                // Failure was ruled out above.
                // As such, the status must be Running. We will continue updating child nodes, but we set "finished" to false in order to make sure we don't signal Success just yet.

                finished = false
            }
        }

        return if(finished) BehaviorStatus.Success
        else BehaviorStatus.Running
    }
}

/**
 * Selector Node (fallback node) Implementation.
 * - Child nodes will be updated in order.
 * - When Running is signaled by a node, the execution stops and the node signals Running.
 * - When Failure is signaled by a node, the execution moves onto the next node *in sequence*.
 * - When Success is signaled by a node, the Selector Node will yield the final status Success.
 * - When no more nodes are available (all child nodes yielded Failure), the final status is Failure. As per the convention, an empty list of child nodes will yield Success.
 *
 * As a clarification, if [runOnce] is set to `true`, and Success is encountered, no further updates will be sent to any of the child nodes.
 * */
class BehaviorSelectorNode(name: String, runOnce: Boolean, children: List<BehaviorNode>) : BehaviorProxyNode(name, runOnce, children) {
    override fun update(context: BehaviorContext): BehaviorStatus {
        if(children.isEmpty()) {
            // Special case: return Success for empty subtree.

            return BehaviorStatus.Success
        }

        children.forEach {
            val status = it.getStatus(context)

            if(status != BehaviorStatus.Failure) {
                return status
            }
        }

        return BehaviorStatus.Failure
    }
}

/**
 * Inverter Node Implementation.
 * - The child node is updated as long as it signals Running.
 * - When Success/Failure is signaled, the Inverter Node will yield the final status Success (if the child node's status is Failure) or Failure (if the child node's status is Success)
 *
 * The Running status **is not transformed** by this node.
 * */
class BehaviorInverterNode(name: String, runOnce: Boolean, child: BehaviorNode) : BehaviorDecoratorNode(name, runOnce, child) {
    override fun transform(status: BehaviorStatus): BehaviorStatus {
        if(status == BehaviorStatus.Running) {
            return BehaviorStatus.Running
        }

        return status.inverse
    }
}

/**
 * [BehaviorDecoratorNode] that transforms a non-running status into [status].
 * [status] must not be [BehaviorStatus.Running].
 * */
class BehaviorResultNode(name: String, runOnce: Boolean, child: BehaviorNode, val status : BehaviorStatus) : BehaviorDecoratorNode(name, runOnce, child) {
    init {
        require(status != BehaviorStatus.Running) { "Cannot create a result node with ${BehaviorStatus.Running} status"}
    }

    override fun transform(status: BehaviorStatus): BehaviorStatus {
        if(status == BehaviorStatus.Running) {
            return BehaviorStatus.Running
        }

        return this.status
    }
}

/**
 * External call node. This is a novel concept:
 * - This node is bound to another behavior node, [target] (which requires a two-pass loading process)
 * - A fresh [BehaviorContext] is created and stored in the context passed down to [update]. This context is fetched and used for subsequent calls.
 * - The [target] node's [BehaviorNode.getStatus] method is called with this separate context.
 *
 * This is effectually similar to re-creating the entire subtree, starting from [target].
 * In that case, the new nodes would store separate states in the [BehaviorContext]'s maps, being separate instances.
 * Instead, we created a separate context for them. See the Addendum at the top of the file for more details.
 * */
class BehaviorCallNode(name: String, runOnce: Boolean) : BehaviorNode(name, runOnce) {
    private var target: BehaviorNode? = null

    fun bind(target: BehaviorNode) {
        require(this.target == null) { "Cannot re-bind call node." }
        this.target = target
    }

    override fun update(context: BehaviorContext): BehaviorStatus {
        val target = this.target
            ?: error("External call $name not bound")

        // Refer to the top of this file for an explanation.
        val callContext = context.getOrStore(CallContextKey(context, this)) { BehaviorContext() }

        return target.getStatus(callContext)
    }

    /**
     * [BehaviorContext.storage] key for the separate [BehaviorContext] passed down to [target].
     * */
    private data class CallContextKey(val parent: BehaviorContext, val node: BehaviorCallNode) {
        override fun equals(other: Any?): Boolean {
            if(other !is CallContextKey) {
                return false
            }

            return parent == other.parent && node == other.node
        }

        override fun hashCode(): Int {
            var result = parent.hashCode()
            result = 31 * result + node.hashCode()
            return result
        }
    }
}

/**
 * Custom leaf nodes implemented using the editor's YAML definition system.
 * */
abstract class BehaviorCompositeNode(name: String, runOnce: Boolean, private val storedData: String) : BehaviorNode(name, runOnce) {
    /**
     * Loads the data stored in this node into [TStorage].
     * All primary constructor parameters of [TStorage] must map to a stored value.
     * */
    protected fun <TStorage : Any> loadStorage(storageClass: KClass<TStorage>): TStorage {
        return Gson().fromJson(storedData, JsonCompositeState::class.java).load(storageClass)
    }

    protected fun <TStorage: Any> loadStorageJava(storageClass: Class<TStorage>) : TStorage {
        return loadStorage(storageClass.kotlin)
    }
}

/**
 * Drive interface for the [BehaviorMotionNode].
 * */
interface IDriveController {
    /**
     * Checks if [trajectory] is the trajectory currently being followed.
     * If no trajectory is being followed, this must return `false`.
     * */
    fun isActualTrajectory(trajectory: Trajectory): Boolean

    /**
     * Gets the displacement along the current trajectory.
     * If no trajectory is being followed, this is expected to produce an error.
     * The intended logic will not call this unless a trajectory is actually being followed.
     * */
    val displacement: Double

    /**
     * Begins following the specified trajectory.
     * If this trajectory is already being followed, the follower's state will be reset.
     * */
    fun beginFollow(trajectory: Trajectory)

    /**
     * True if the follower has reached the destination.
     * If no trajectory is being followed, this is expected to produce an error.
     * The intended logic will not call this unless a trajectory is actually being followed.
     * */
    val isAtDestination: Boolean
}

/**
 * Motion node. It is a bit unconventional in terms of Behavior Tree architecture.
 * It is similar to an *Action* node, but also acts as a *Proxy* node (the markers).
 *
 * The "fundamental node" is the *Proxy*-like behavior of this node. It is either a [BehaviorSequenceNode] or a [BehaviorParallelNode].
 * */
class BehaviorMotionNode(createContext: BehaviorCreateContext, project: JsonProject, private val follower: IDriveController) : BehaviorNode(createContext.name, createContext.runOnce), IParentBehavior {
    private data class JsonBinding(
        val TerminalId: Int,
        val Marker: String
    )

    private enum class JsonNodeType {
        @SerializedName("0")
        Sequence,
        @SerializedName("1")
        Parallel
    }

    private data class JsonState(
        val MotionProject: String,
        val Bindings: List<JsonBinding>,
        val Type: JsonNodeType
    )

    private data class MarkerListener(val marker: TrajectoryMarker, val nodes: List<BehaviorNode>)

    /**
     * *Ghost* child node for [fundamentalNode].
     * - If the marker is not hit yet, [BehaviorStatus.Running] is returned.
     * - If the marker was hit, the status of [boundChild] is returned.
     * */
    private class MarkerNode(name: String, runOnce: Boolean, val listener: MarkerListener, val boundChild: BehaviorNode, val key: StorageKey) : BehaviorNode(name, runOnce) {
        override fun update(context: BehaviorContext): BehaviorStatus {
            // P.S. it is guaranteed the stored data exists,
            // since BehaviorMotionNode is updated first (and the intended logic will create and update this stored value)
            val storedData = context.getStored<StorageKey, StorageValue>(key)

            if(!storedData.fired.contains(listener)) {
                // In this case, return Running (waiting for marker to hit)

                return BehaviorStatus.Running
            }

            // This marker has fired, so it is time to delegate call to actual the child:
            return boundChild.getStatus(context)
        }
    }

    // This is not an execution variable, because the list and content (MarkerListener is immutable) do not change during execution.
    private val listeners: List<MarkerListener>

    // This is not an execution variable, because it is immutable:
    private val trajectory: Trajectory

    /**
     * The *fundamental node* is a behavior node selected based on [JsonNodeType]
     * It wraps all nodes held by the markers.
     *
     * It is also not an execution variable. It is akin to a proxy node's child node.
     * */
    private val fundamentalNode: BehaviorNode

    override val children: List<BehaviorNode>
        get() = listOf(fundamentalNode)

    init {
        val state = Gson().fromJson(createContext.savedData, JsonState::class.java)

        val motionProject = project.MotionProjects[state.MotionProject]
            ?: error("Failed to fetch motion project ${state.MotionProject} for node ${createContext.name}")

        val editorTrajectory = loadTrajectory(motionProject)
        trajectory = editorTrajectory.trajectory

        // Map the loaded children in "createContext" to the children we have in "state":

        val listeners = ArrayList<MarkerListener>()
        state.Bindings.forEach { jsonBinding ->
            val boundChildren = createContext.children[jsonBinding.TerminalId]
                ?: error("Failed to bind stored children ${jsonBinding.TerminalId}, in marker ${jsonBinding.Marker} to actual children")

            // "children" is the nodes that are listening on this marker.
            // now, we will fetch the marker from the list:

            val marker = editorTrajectory.markers.firstOrNull { it.name == jsonBinding.Marker }
                ?: error("Failed to get trajectory marker ${jsonBinding.Marker}")

            listeners.add(MarkerListener(marker, boundChildren))
        }

        this.listeners = listeners

        // Now, to create the "fundamental node", we need to create the wrapper (ghost) nodes for every listener child:
        val ghostNodes = listeners.map { listener ->
            listener.nodes.map { node ->
                MarkerNode(
                    name = listener.marker.name,
                    runOnce = true,
                    listener,
                    boundChild = node,
                    key = StorageKey(this)
                )
            }
        }.flatten()

        // Then, select the implementation:
        fundamentalNode = when(state.Type) {
            JsonNodeType.Sequence ->
                BehaviorSequenceNode(
                    name = "${createContext.name} Sequence",
                    runOnce = true,
                    children = ghostNodes
                )
            JsonNodeType.Parallel ->
                BehaviorParallelNode(
                    name = "${createContext.name} Parallel",
                    runOnce = true,
                    children = ghostNodes
                )
        }
    }

    override fun update(context: BehaviorContext): BehaviorStatus {
        // First, check if the drive is set to follow this trajectory:

        if(!follower.isActualTrajectory(trajectory)) {
            // If not, begin following:
            follower.beginFollow(trajectory)
        }

        // We will get the listeners that have been fired.
        // A listener has fired if the trajectory's current displacement is larger or equal to the marker's.
        val actualDisplacement = follower.displacement

        // We store the fired state in the context, for the ghost nodes to access:
        context.getOrStore(StorageKey(this)) { StorageValue(HashSet()) }
            .also {
                it.fired.clear() // This needs to be researched more. If we find that "displacement" should be strictly monotonous increasing,
                // then clearing does not make sense (if displacement goes backwards for some reason, we will get intermittent child updates)

                it.fired.addAll(
                    listeners.filter { l ->
                        actualDisplacement >= l.marker.point.displacement
                    }
                )
            }

        // First, we select a behavior status based on the follower's state.
        // This is pretty simple, it is Running if we haven't reached the destination, and Success if we have:
        val followState =
            if(follower.isAtDestination) BehaviorStatus.Success
            else BehaviorStatus.Running

        // Now, get the status of the fundamental node (and consequently of the child nodes):
        val markerState = fundamentalNode.getStatus(context)

        // Then, we compose the states using AND.
        // This operation is appropriate here for the following reasons:
        // - If we aren't at the destination, "followState" will be Running. AND will yield Running if the marker state is Running or Success.
        // - If the marker trees haven't finished running, "markerState" will be Running. AND will yield Running if the follower state is Running or Success.
        // - If a state is Failure, the total state of this node will be Failure.
        return BehaviorStatus.and(followState, markerState)
    }

    /**
     * Storage key for state storage. Refer to the document at the top of this file to see why this is required.
     * */
    private data class StorageKey(val motionNode: BehaviorMotionNode) {
        override fun equals(other: Any?): Boolean {
            if(other !is StorageKey) {
                return false
            }

            return motionNode == other.motionNode
        }

        override fun hashCode(): Int {
            return motionNode.hashCode()
        }
    }

    /**
     * The execution state of this node, stored in the [BehaviorContext].
     * @param fired The set of markers containing all [MarkerListener]s that have fired.
     * */
    private data class StorageValue(val fired: HashSet<MarkerListener>)
}