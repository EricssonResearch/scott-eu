import sbt.project

ThisBuild / scalaVersion := "2.12.6"

//
//lazy val hello = (project in file("."))
//    .settings(
//        name := "Orchestrator",
//        libraryDependencies += ,
//)

resolvers += Resolver.mavenLocal

libraryDependencies ++= Seq(
    "se.ericsson.cf.scott.sandbox" % "domain-pddl" % "0.0.1-SNAPSHOT",
    "org.slf4j" % "slf4j-simple" % "1.7.25",
    "com.typesafe.scala-logging" %% "scala-logging" % "3.9.0"
)
