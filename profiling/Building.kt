package com.roshanah.jerky.profiling

import com.roshanah.jerky.math.Pose
import com.roshanah.jerky.math.Angle
import com.roshanah.jerky.math.Vec2
import com.roshanah.jerky.math.newtonMethodSolve
import com.roshanah.jerky.utils.DriveConstants
import kotlin.math.abs
import kotlin.math.sign


class ProfileBuilder internal constructor(val constants: DriveConstants, val xi: Pose = Pose.zero, val vi: Pose = Pose.zero) {
  val profiles = mutableListOf<Profile>()
  val vf: Pose
    get() = if (profiles.isEmpty()) vi else profiles.last().vf

  fun interpolate(vi: Pose, vf: Pose) = constants.run{
    val viConstrained = constrain(vi)
    val vfConstrained = constrain(vf)

    val a = findAcceleration(viConstrained, vfConstrained)
    val diff = vfConstrained - viConstrained

    val headingLength = abs(diff.heading / a.heading)
    val translateLength = diff.pos.magnitude / a.pos.magnitude

    val length =
        if (headingLength.isFinite()) headingLength
        else if (translateLength.isFinite()) translateLength else 0.0

    Profile(Pose.zero, viConstrained, a, length, this)
  }

  fun to(vf: Pose) = interpolate(this.vf, vf).also { profiles.add(it) }
  fun to(vx: Double, vy: Double, w: Double) = to(Pose(vx, vy, w))


  fun displacement(vi: Pose, distance: Double) = constants.constrain(vi).run {
    Profile(Pose.zero, this, Pose.zero, distance / this.pos.magnitude, constants)
  }
  fun displace(distance: Double) = displacement(this.vf, distance).also { profiles.add(it) }

  fun displacement(vi: Pose, dTheta: Angle) = constants.constrain(vi).run{
    Profile(Pose.zero, this, Pose.zero, abs(dTheta.rad / this.heading), constants)
  }
  fun displace(dTheta: Angle) = displacement(this.vf, dTheta).also { profiles.add(it) }

  fun interpolateAndTurn(vi: Vec2, vf: Vec2, dTheta: Angle): CompoundProfile{
    val halfway = (vi + vf) * 0.5

    // val maxW = (maxAcceleration / halfway.magnitude).coerceAtMost((maxVelocity - halfway.magnitude * sqrt(2.0)) / (2 * trackRadius))

    val displacementFun: (Double) -> Double = {
      val speedUp = interpolate(Pose(vi, 0.0), Pose(halfway, it * sign(dTheta.rad)))
      val slowDown = interpolate(speedUp.vf, Pose(vf, 0.0))
      abs(speedUp.displacement.heading) + abs(slowDown.displacement.heading) - abs(dTheta.rad) 
    }

    val w = newtonMethodSolve(
      displacementFun,
      constants.maxVelocity * 0.5
    )

    // return if (true){ // TODO fix this bug
    //   val speedUp = interpolateVelocities(Pose(vi, 0.0), Pose(halfway, w * sign(dTheta.rad)))
    //   val slowDown = interpolateVelocities(speedUp.vf, Pose(vf, 0.0))
    //   speedUp + slowDown
    // }else{
    //   val speedUp = interpolateVelocities(Pose(vi, 0.0), Pose(halfway, maxW * sign(dTheta.rad)))
    //   val slowDown = interpolateVelocities(speedUp.vf, Pose(vf, 0.0))
    //   val displacement = abs(dTheta.rad) - abs(speedUp.displacement.heading) - abs(slowDown.displacement.heading)
    //
    //   val maintain = displacement(speedUp.vf, displacement)
    //
    //   speedUp + maintain + slowDown
    // }

    val speedUp = interpolate(Pose(vi, 0.0), Pose(halfway, w * sign(dTheta.rad)))
    val slowDown = interpolate(speedUp.vf, Pose(vf, 0.0))
    return speedUp + slowDown
  }
  fun turnTo(vf: Vec2, dTheta: Angle) = interpolateAndTurn(this.vf.pos, vf, dTheta).also { profiles.addAll(it.profiles) }
  fun turnTo(vx: Double, vy: Double, dTheta: Angle) = turnTo(Vec2(vx, vy), dTheta)

  fun stop() = to(Pose.zero)

  fun append(profile: Profile) = profiles.add(profile)
  fun append(profile: CompoundProfile) = profiles.addAll(profile.profiles)

  fun build() = CompoundProfile(profiles)
}

fun buildProfile(constants: DriveConstants, xi: Pose = Pose.zero, vi: Pose = Pose.zero, builder: ProfileBuilder.() -> Unit): CompoundProfile{
  val profileBuilder = ProfileBuilder(constants)
  profileBuilder.builder()
  return profileBuilder.build()
}

