package com.roshanah.jerky.profiling

import com.roshanah.jerky.math.Linear

data class Derivatives<T : Linear<T>>(val pos: T, val vel: T, val accel: T): Linear<Derivatives<T>>{
  override fun plus(other: Derivatives<T>) = Derivatives(pos + other.pos, vel + other.vel, accel + other.accel)
  override fun minus(other: Derivatives<T>) = Derivatives(pos - other.pos, vel - other.vel, accel - other.accel)
  override fun times(scalar: Double) = Derivatives(pos * scalar, vel * scalar, accel * scalar)
}
