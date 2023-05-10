import com.google.gson.Gson
import empireu.coyote.*

enum class Options {
    Option1,
    Option2
}

data class Container(
    val myFlag: Boolean,
    val myFlag2: Boolean,
    val myOptions: Options,
    val myRealSlider: Double,
    val mySlider: Int,
    val myText: String,
    val myRealInput: Double
)

fun main(args: Array<String>) {

    val json = "{\u0022Flags\u0022:[{\u0022State\u0022:true,\u0022Name\u0022:\u0022myFlag\u0022,\u0022Label\u0022:\u0022My Flag\u0022},{\u0022State\u0022:true,\u0022Name\u0022:\u0022myFlag2\u0022,\u0022Label\u0022:\u0022\u0022}],\u0022Enums\u0022:[{\u0022Options\u0022:[\u0022Option1\u0022,\u0022Option2\u0022],\u0022Selected\u0022:\u0022Option2\u0022,\u0022Name\u0022:\u0022myOptions\u0022,\u0022Label\u0022:\u0022\u0022}],\u0022RealSliders\u0022:[{\u0022Min\u0022:-1,\u0022Max\u0022:2,\u0022Value\u0022:0.5,\u0022Name\u0022:\u0022myRealSlider\u0022,\u0022Label\u0022:\u0022My Real Slider\u0022}],\u0022IntegerSliders\u0022:[{\u0022Min\u0022:-10,\u0022Max\u0022:10,\u0022Value\u0022:-6,\u0022Name\u0022:\u0022mySlider\u0022,\u0022Label\u0022:\u0022My Integer Slider\u0022}],\u0022TextInputFields\u0022:[{\u0022MaxLength\u0022:512,\u0022Value\u0022:\u0022tex\u0022,\u0022Name\u0022:\u0022myText\u0022,\u0022Label\u0022:\u0022\u0022}],\u0022RealInputFields\u0022:[{\u0022Step\u0022:0.01,\u0022Value\u0022:0.01,\u0022Name\u0022:\u0022myRealInput\u0022,\u0022Label\u0022:\u0022\u0022}],\u0022IntegerInputFields\u0022:[{\u0022Step\u0022:1,\u0022Value\u0022:4,\u0022Name\u0022:\u0022myIntegerInput\u0022,\u0022Label\u0022:\u0022\u0022}],\u0022IsDriveBehavior\u0022:false,\u0022IsNonParallel\u0022:true}"
    val state = Gson().fromJson(json, JsonCompositeState::class.java).load(Container::class)

    val project = loadCoyoteProject()

    val t = loadTrajectory(project.MotionProjects["My Project"]!!)

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