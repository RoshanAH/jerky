package com.roshanah.jerky.profiling

import com.roshanah.jerky.math.Angle
import com.roshanah.jerky.math.Mat2
import com.roshanah.jerky.math.Pose
import com.roshanah.jerky.math.Vec2
import com.roshanah.jerky.math.cos
import com.roshanah.jerky.math.rad
import com.roshanah.jerky.math.rotation
import com.roshanah.jerky.math.sin
import com.roshanah.jerky.math.quadratic
import com.roshanah.jerky.math.deg
import com.roshanah.jerky.math.newtonMethodSolve
import com.roshanah.jerky.utils.DriveConstants
import com.roshanah.jerky.utils.DriveValues
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sign
import kotlin.math.sqrt

data class ProfilePoint(
    val motion: Derivatives<Pose>,
    val wheels: Derivatives<DriveValues>,
) {
  companion object {
    val zero: ProfilePoint =
        ProfilePoint(
            Derivatives(Pose.zero, Pose.zero, Pose.zero),
            Derivatives(DriveValues.zero, DriveValues.zero, DriveValues.zero)
        )
  }
}

interface Bounded : (Double) -> ProfilePoint {
  val length: Double
  val start: ProfilePoint
    get() = this(0.0)
  val end: ProfilePoint
    get() = this(length)
  val displacement: Pose
    get() = end.motion.pos - start.motion.pos
  val vf: Pose
    get() = end.motion.vel

  override fun invoke(t: Double) = if (t < 0.0) start else if (t > length) end else boundedInvoke(t)
  fun boundedInvoke(t: Double): ProfilePoint
  fun continous(other: Bounded): Boolean {
    val last = end.motion
    val difference = last.vel - other.start.motion.vel
    val error = difference.pos.magnitude + abs(difference.heading)
    return error <= 1e-10
  }
}

interface Offsettable<T : Bounded> : Bounded {
  fun offset(xi: Pose): T
}

class Profile
internal constructor(
    val xi: Pose,
    val vi: Pose,
    val a: Pose,
    override val length: Double,
    val constants: DriveConstants
) : Offsettable<Profile> {

  override fun boundedInvoke(t: Double): ProfilePoint {

    val motion =
        Derivatives(
            a * 0.5 * t.pow(2.0) + vi * t + xi,
            a * t + vi,
            a,
        )

    val theta = motion.pos.heading.rad
    val w = motion.vel.heading
    val alpha = motion.accel.heading

    val rot = rotation(theta) // rotation matrix
    val rotPrime =
        Mat2(-sin(theta), cos(theta), -cos(theta), -sin(theta)) * w // derivative of rotation matrix

    val rv = rot * motion.vel.pos
    val ra = rotPrime * motion.vel.pos + rot * motion.accel.pos // product rule

    val r = constants.trackRadius

    val wheels =
        Derivatives(
            DriveValues(
                0.0,
                0.0,
                0.0,
                0.0
            ), // this integral is impossible to evaluate manually and really not useful so we'll
            // just put zeros here
            DriveValues( // inverse kinematics for velocity
                rv.y + rv.x - (2 * r * w),
                rv.y - rv.x + (2 * r * w),
                rv.y - rv.x - (2 * r * w),
                rv.y + rv.x + (2 * r * w),
            ),
            DriveValues( // derivative of our velocity
                ra.y + ra.x - (2 * r * alpha),
                ra.y - ra.x + (2 * r * alpha),
                ra.y - ra.x - (2 * r * alpha),
                ra.y + ra.x + (2 * r * alpha),
            ),
        )

    return ProfilePoint(motion, wheels)
  }

  override fun offset(xi: Pose) = Profile(this.xi + xi, vi, a, length, constants)

  operator fun plus(other: Profile): CompoundProfile {
    val last = end.motion
    val difference = last.vel - other.start.motion.vel
    val error = difference.pos.magnitude + abs(difference.heading)
    if (error > 1e-10) throw ProfileContinuetyException("error: $error")

    return CompoundProfile(listOf(this, other.offset(last.pos - other.xi)))
  }

  operator fun plus(other: CompoundProfile): CompoundProfile {
    val last = end.motion
    val difference = last.vel - other.start.motion.vel
    val error = difference.pos.magnitude + abs(difference.heading)
    if (error > 1e-10) throw ProfileContinuetyException()

    return CompoundProfile(
        listOf(this) + other.offset(end.motion.pos - other.start.motion.pos).profiles
    )
  }

}

class CompoundProfile internal constructor(profiles: List<Profile>) :
    Offsettable<CompoundProfile> {
  override val length = profiles.sumOf { it.length }
  val profiles: List<Profile>

  init {
    for(i in 0 until profiles.size - 1){
      if (!profiles[i].continous(profiles[i + 1])) throw ProfileContinuetyException()
    }

    this.profiles = buildList{
      if (profiles.isEmpty()) throw IllegalArgumentException("Compound profile cannot be empty")
      add(profiles[0])
      for(i in 1 until profiles.size){
        val p = profiles[i]
        add(p.offset(profiles[i - 1].xi - p.xi))
      }
    }
  }

  // returns the sub-profile at t
  operator fun get(t: Double): Profile {
    if (t < 0.0) return profiles.first()

    var remaining = t
    for (p in profiles) {
      if (remaining < p.length) return p
      remaining -= p.length
    }

    return profiles.last()
  }

  override fun boundedInvoke(t: Double): ProfilePoint {
    if (t < 0.0) return profiles.first().start

    var remaining = t
    for (p in profiles) {
      if (remaining < p.length) return p(remaining)
      remaining -= p.length
    }

    return profiles.last().end
  }

  override fun offset(xi: Pose) = CompoundProfile(profiles.map { it.offset(xi) })

  operator fun plus(other: Profile): CompoundProfile {
    val last = end.motion
    val difference = last.vel - other.start.motion.vel
    val error = difference.pos.magnitude + abs(difference.heading)
    if (error > 1e-10) throw ProfileContinuetyException()

    return CompoundProfile(profiles + other.offset(last.pos - other.xi))
  }

  operator fun plus(other: CompoundProfile): CompoundProfile {
    val last = end.motion
    val difference = last.vel - other.start.motion.vel
    val error = difference.pos.magnitude + abs(difference.heading)
    if (error > 1e-10) throw ProfileContinuetyException("difference of $error")

    return CompoundProfile(
        profiles + other.offset(last.pos - other.start.motion.pos).profiles
    )
  }
}

class ProfileContinuetyException(msg: String) : IllegalArgumentException(msg){
  constructor(): this("")
}
