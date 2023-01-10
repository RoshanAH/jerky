package com.roshanah.jerky.utils

import com.roshanah.jerky.math.Pose
import com.roshanah.jerky.math.Vec2
import com.roshanah.jerky.math.rotation
import com.roshanah.jerky.math.sin
import com.roshanah.jerky.math.cos
import com.roshanah.jerky.math.Mat2
import com.roshanah.jerky.profiling.Derivatives
import com.roshanah.jerky.math.rad
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sign
import kotlin.math.sqrt

data class PSVAConstants(val kP: Double, val kS: Double, val kV: Double, val kA: Double)

data class DriveConstants(
    val maxVelocity: Double,
    val maxAcceleration: Double,
    val trackRadius: Double,
    val yPSVA: PSVAConstants,
    val xPSVA: PSVAConstants,
    val rPSVA: PSVAConstants,
    val wheelScalar: DriveValues,
) {

  constructor(maxVelocity: Double, maxAcceleration: Double, trackRadius: Double, psva: PSVAConstants) : 
    this(maxVelocity, maxAcceleration, trackRadius, psva, psva, psva, DriveValues(1.0, 1.0, 1.0, 1.0))

 fun wheels(motion: Derivatives<Pose>): Derivatives<DriveComponents>{

    val theta = motion.pos.heading.rad
    val w = motion.vel.heading
    val alpha = motion.accel.heading

    val rot = rotation(-theta) // rotation matrix
    val rotPrime =
        Mat2(sin(theta), cos(theta), -cos(theta), sin(theta)) * w // derivative of rotation matrix

    val rv = rot * motion.vel.pos
    val ra = rotPrime * motion.vel.pos + rot * motion.accel.pos // product rule

    val r = trackRadius

    return Derivatives(
      DriveComponents.zero,
      DriveComponents(
        DriveValues(1.0, 1.0, 1.0, 1.0) * rv.y,
        DriveValues(1.0, -1.0, -1.0, 1.0) * rv.x,
        DriveValues(-1.0, 1.0, -1.0, 1.0) * 2.0 * r * w,
      ),
      DriveComponents(
        DriveValues(1.0, 1.0, 1.0, 1.0) * ra.y,
        DriveValues(1.0, -1.0, -1.0, 1.0) * ra.x,
        DriveValues(-1.0, 1.0, -1.0, 1.0) * 2.0 * r * alpha,
      ),
    )
  }

  fun wheelsRelative(motion: Derivatives<Pose>): Derivatives<DriveComponents> {

    val w = motion.vel.heading
    val alpha = motion.accel.heading

    val r = trackRadius

    val v = motion.vel.pos
    val a = motion.accel.pos

    return Derivatives(
      DriveComponents.zero,
      DriveComponents(
        DriveValues.forward * v.y,
        DriveValues.strafe * v.x,
        DriveValues.turn * 2.0 * r * w,
      ),
      DriveComponents(
        DriveValues.forward * a.y,
        DriveValues.strafe * a.x,
        DriveValues.turn * 2.0 * r * alpha,
      ),
    )

  }


  private fun unscaledSva(components: Derivatives<DriveComponents>): DriveValues {
    val vel = components.vel.powers(yPSVA.kV, xPSVA.kV, rPSVA.kV)    
    val accel = components.accel.powers(yPSVA.kA, xPSVA.kA, rPSVA.kA)    
    val static = (components.vel + components.accel).run { forward.sign * yPSVA.kS + strafe.sign * xPSVA.kS + turn.sign * rPSVA.kS }
    return static + vel + accel
  }

  fun sva(components: Derivatives<DriveComponents>) = unscaledSva(components) * wheelScalar

  @JvmName("svaFromMotion")
  fun sva(motion: Derivatives<Pose>) = unscaledSva(wheels(motion)) * wheelScalar

  fun psva(components: Derivatives<DriveComponents>, vel: Pose): DriveValues{
    val measured = DriveComponents(
        DriveValues.forward * vel.y,
        DriveValues.strafe * vel.x,
        DriveValues.turn * 2.0 * trackRadius * vel.heading,
    )

    return ((components.vel - measured).powers(yPSVA.kP, xPSVA.kP, rPSVA.kP) + unscaledSva(components)) * wheelScalar
  }
  @JvmName("psvaFromMotion")
  fun psva(motion: Derivatives<Pose>, vel: Pose) = psva(wheels(motion), vel)

  fun constrain(velocity: Pose): Pose {
    val velocityConstraint =
        (velocity.pos.magnitude * sqrt(2.0) + 2 * trackRadius * abs(velocity.heading)) / maxVelocity
    val accelConstraint =
        sqrt((velocity.pos * velocity.heading).magnitude * sqrt(2.0) / maxAcceleration)
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
    if (difference.pos.magnitude == 0.0)
        return findAcceleration(viConstrained, vfConstrained.heading)

    // we solve for acceleration in terms of alpha assuming we want to finish accelerating at the
    // same
    // time
    val relation = (difference.pos).magnitude / abs(difference.heading)

    val wMax: Double
    val vMax: Vec2

    if (viConstrained.run { pos * heading }.magnitude >
            vfConstrained.run { pos * heading }.magnitude
    ) {
      wMax = abs(viConstrained.heading)
      vMax = viConstrained.pos
    } else {
      wMax = abs(vfConstrained.heading)
      vMax = vfConstrained.pos
    }

    val alpha = (maxAcceleration - wMax * vMax.magnitude) / (relation * sqrt(2.0) + 2 * trackRadius)
    val acceleration =
        alpha * relation // this is because we solved for alpha in terms of acceleration

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
