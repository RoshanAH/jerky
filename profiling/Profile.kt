package com.roshanah.jerky.profiling

import com.roshanah.jerky.math.*
import com.roshanah.jerky.utils.DriveConstants
import com.roshanah.jerky.utils.DriveValues
import com.roshanah.jerky.profiling.Derivatives
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sign
import kotlin.math.sqrt

typealias MotionPoint = Derivatives<Pose>

interface Bounded : (Double) -> Derivatives<Pose> {
  val length: Double
  val start: MotionPoint
    get() = this(0.0)
  val end: MotionPoint
    get() = this(length)
  val displacement: Pose
    get() = end.pos - start.pos
  val vf: Pose
    get() = end.vel
  val vi: Pose
    get() = start.vel

  override fun invoke(t: Double) = if (t < 0.0) start else if (t > length) end else boundedInvoke(t)

  fun boundedInvoke(t: Double): MotionPoint

  fun continous(other: Bounded): Boolean {
    val difference = vf - other.vi
    val error = difference.pos.magnitude + abs(difference.heading)
    // println("last: ${vf} start: ${other.vi}")
    return error <= 1e-10
  }
}

interface Offsettable<T : Bounded> : Bounded {
  fun offset(xi: Pose): T
}

class Profile
internal constructor(
    val xi: Pose,
    override val vi: Pose,
    val a: Pose,
    override val length: Double
) : Offsettable<Profile> {

  override fun boundedInvoke(t: Double) = 
    Derivatives(
        a * 0.5 * t.pow(2.0) + vi * t + xi,
        a * t + vi,
        a,
    )

  override fun offset(xi: Pose) = Profile(this.xi + xi, vi, a, length)

  operator fun plus(other: Profile): CompoundProfile {
    val last = end
    return CompoundProfile(listOf(this, other.offset(last.pos - other.xi)))
  }
  operator fun plus(other: CompoundProfile) = CompoundProfile( listOf(this) + other.offset(end.pos - other.start.pos).profiles)
  
  companion object {
    val stopped = Profile(Pose.zero, Pose.zero, Pose.zero, Double.POSITIVE_INFINITY)
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
        add(p.offset(this[i - 1].end.pos - p.xi))
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

  override fun boundedInvoke(t: Double): MotionPoint {
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
    val last = end
    return CompoundProfile(profiles + other.offset(last.pos - other.xi))
  }

  operator fun plus(other: CompoundProfile): CompoundProfile {
    val last = end

    return CompoundProfile(
        profiles + other.offset(last.pos - other.start.pos).profiles
    )
  }
}

class ProfileContinuetyException(msg: String) : IllegalArgumentException(msg){
  constructor(): this("")
}
