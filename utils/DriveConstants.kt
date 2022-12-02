package com.roshanah.jerky.utils

import com.roshanah.jerky.profiling.Derivatives
import com.roshanah.jerky.math.Pose
import com.roshanah.jerky.math.Vec2
import kotlin.math.sqrt
import kotlin.math.pow
import kotlin.math.sign
import kotlin.math.abs

data class PSVAConstants(
  val kP: Double,
  val kS: Double,
  val kV: Double,
  val kA: Double
)

data class DriveConstants(
  val maxVelocity: Double,
  val maxAcceleration: Double,
  val trackRadius: Double,
  val psva: PSVAConstants
) {
  fun sva(wheels: Derivatives<DriveValues>) = (if (wheels.vel.sign == DriveValues.zero) wheels.accel.sign else wheels.vel.sign) * psva.kS + wheels.vel * psva.kV + wheels.accel * psva.kA
  fun psva(wheels: Derivatives<DriveValues>, velocity: DriveValues) = sva(wheels) + (wheels.vel - velocity) * psva.kP

  fun constrain(velocity: Pose): Pose {
    val velocityConstraint = (velocity.pos.magnitude * sqrt(2.0) + 2 * trackRadius * abs(velocity.heading)) / maxVelocity
    val accelConstraint = sqrt((velocity.pos * velocity.heading).magnitude * sqrt(2.0) / maxAcceleration)
    // println("accel constraint: $accelConstraint vel constraint: $velocityConstraint")

    return velocity / 1.0.coerceAtLeast(velocityConstraint).coerceAtLeast(accelConstraint)
  }

  fun findAcceleration(vi: Pose, vf: Pose): Pose {
    val viConstrained = constrain(vi)
    val vfConstrained = constrain(vf)

    val difference = vfConstrained - viConstrained
    val ahat = difference.pos.unit // this is the direction we want to accelerate in
    val sa = sign(difference.heading)


    if (difference.heading == 0.0) return findAcceleration(viConstrained, vfConstrained.pos)
    if (difference.pos.magnitude == 0.0) return findAcceleration(viConstrained, vfConstrained.heading)

    // we solve for acceleration in terms of alpha assuming we want to finish accelerating at the same
    // time
    val relation = (difference.pos).magnitude / abs(difference.heading)

    val wMax: Double
    val vMax: Vec2

    if (viConstrained.run{pos * heading}.magnitude > vfConstrained.run{pos * heading}.magnitude){
      wMax = abs(viConstrained.heading)
      vMax = viConstrained.pos
    } else {
      wMax = abs(vfConstrained.heading)
      vMax = vfConstrained.pos
    }

    val alpha = (maxAcceleration - wMax * vMax.magnitude) / (relation * sqrt(2.0) + 2 * trackRadius)
    val acceleration = alpha * relation // this is because we solved for alpha in terms of acceleration

    return Pose(ahat * acceleration, alpha * sa)
  }

  // no change in angular velocity
  fun findAcceleration(vi: Pose, vf: Vec2): Pose {
    val initial = vi.pos.square * vi.heading.pow(2.0)
    val final = vf.square * vi.heading.pow(2.0)
    val max = initial.coerceAtLeast(final)
    return Pose((vf - vi.pos).normalize() * sqrt(maxAcceleration.pow(2.0) * 0.5 - max), 0.0)
  }

  // no change in translation velocity
  fun findAcceleration(vi: Pose, vf: Double): Pose {
    val initial = vi.pos.magnitude * abs(vi.heading)
    val final = vi.pos.magnitude * abs(vf)
    val max = initial.coerceAtLeast(final)
    return Pose(Vec2.zero, (maxAcceleration - sqrt(2.0) * 0.5 * max) * sign(vf - vi.heading))
  }
}
