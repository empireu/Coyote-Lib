plugins {
    kotlin("jvm") version "1.8.0"
    application
    `maven-publish`
}

version = "1.0-SNAPSHOT"

repositories {
    mavenCentral()
}

dependencies {
    implementation("com.google.code.gson:gson:2.10.1")
    testImplementation(kotlin("test"))
    implementation(kotlin("reflect"))
}

tasks.test {
    useJUnitPlatform()
}

kotlin {
    jvmToolchain(8)
}

application {
    mainClass.set("MainKt")
}

publishing {
    publications {
        create<MavenPublication>("maven") {
            groupId = "com.github.empireu"
            artifactId = "coyote-lib"
            version = "1.0"

            from(components["java"])
        }
    }
}