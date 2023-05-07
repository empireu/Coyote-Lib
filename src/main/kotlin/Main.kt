import empireu.coyote.*

fun main(args: Array<String>) {
    val project = loadCoyoteProject()

    val nodeProject = loadNodeProject(project.NodeProjects["My Project"]!!.RootNodes, BehaviorMapBuilder().also { b ->
        b.add("Sequence", { ctx -> BehaviorSequenceNode(ctx.name, ctx.runOnce, ctx.childNodes) })
        b.add("Selector", { ctx -> BehaviorSelectorNode(ctx.name, ctx.runOnce, ctx.childNodes) })
        b.add("Success", { ctx -> BehaviorResultNode(ctx.name, ctx.runOnce, ctx.one(), BehaviorStatus.Success) })
        b.add("Parallel", { ctx -> BehaviorParallelNode(ctx.name, ctx.runOnce, ctx.childNodes) })

        b.add(
            "Call",
            { ctx -> BehaviorCallNode(ctx.name, ctx.runOnce) },

            // The call nodes need a second pass (to search for the target node):
            { ctx ->
                val target = ctx.project.behaviors.firstOrNull { it.root.name == ctx.createContext.savedData }
                    ?: error("Failed to bind call node")

                ctx.node.bind(target.root)
            }
        )
    }.build())
}