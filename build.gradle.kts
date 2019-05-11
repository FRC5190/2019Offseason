import edu.wpi.first.gradlerio.GradleRIOPlugin
import edu.wpi.first.gradlerio.frc.FRCJavaArtifact
import edu.wpi.first.gradlerio.frc.RoboRIO
import edu.wpi.first.toolchain.NativePlatforms
import org.jetbrains.kotlin.gradle.tasks.KotlinCompile

plugins {
    kotlin("jvm") version "1.3.21"
    id("edu.wpi.first.GradleRIO") version "2019.4.1"
}

val kMainRobotClass = "org.ghrobotics.frc2019.RobotKt"

val roborioTargetName = "roborio"

deploy {
    targets {
        // Add the RoboRIO as a target
        target(roborioTargetName, RoboRIO::class.java, closureOf<RoboRIO> {
            team = 5190
        })
    }
    artifacts {
        // Send the JAR to the RoboRIO
        artifact("frcJava", FRCJavaArtifact::class.java, closureOf<FRCJavaArtifact> {
            targets.add(roborioTargetName)
            jvmArgs = listOf(
                "-Xmx20M",
                "-XX:+UseG1GC"
//                "-Dcom.sun.management.jmxremote=true",
//                "-Dcom.sun.management.jmxremote.port=1099",
//                "-Dcom.sun.management.jmxremote.local.only=false",
//                "-Dcom.sun.management.jmxremote.ssl=false",
//                "-Dcom.sun.management.jmxremote.authenticate=false",
//                "-Djava.rmi.server.hostname=10.51.90.2"
            )
        })
    }
}

repositories {
    mavenLocal()
    jcenter()
    maven { setUrl("http://dl.bintray.com/kyonifer/maven") }
    maven { setUrl("https://jitpack.io") }
}

dependencies {
    // Kotlin Standard Library and Coroutines
    compile(kotlin("stdlib"))
    compile("org.jetbrains.kotlinx", "kotlinx-coroutines-core", "1.1.1")

    // FalconLibrary
    compile("org.ghrobotics", "FalconLibrary", "2019.5.12")

    // Apache Commons Math
    compile("org.apache.commons", "commons-math3", "3.6.1")

    // WPILib and Vendors
    wpi.deps.wpilib().forEach { compile(it) }
    wpi.deps.vendor.java().forEach { compile(it) }
    wpi.deps.vendor.jni(NativePlatforms.roborio).forEach { nativeZip(it) }
    wpi.deps.vendor.jni(NativePlatforms.desktop).forEach { nativeDesktopZip(it) }

    // Gson
    compile("com.github.salomonbrys.kotson", "kotson", "2.5.0")

    // Serial Communication
    compile("com.fazecast", "jSerialComm", "2.4.1")

    // XChart for Simulations and Tests
    testCompile("org.knowm.xchart", "xchart", "3.2.2")

    // Unit Testing
    testCompile("junit", "junit", "4.12")
}

tasks.jar {
    doFirst {
        from(configurations.compile.get().map {
            if (it.isDirectory) it else zipTree(it)
        })
        manifest(GradleRIOPlugin.javaManifest(kMainRobotClass))
    }
}

tasks {
    withType<Wrapper>().configureEach {
        gradleVersion = "5.0"
    }
    withType<KotlinCompile>().configureEach {
        kotlinOptions {
            jvmTarget = "1.8"
            freeCompilerArgs += "-Xjvm-default=compatibility"
        }
    }
}
