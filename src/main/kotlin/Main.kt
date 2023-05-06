import empireu.coyote.loadCoyoteProject
import empireu.coyote.loadTrajectory
import empireu.coyote.rounded

fun main(args: Array<String>) {
    val project = loadCoyoteProject()

    val traje = loadTrajectory(project.MotionProjects["My Project"]!!)
}