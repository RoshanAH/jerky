package com.roshanah.jerky.math

data class Pose(val pos: Vec2, val heading: Double) : Linear<Pose>{
  constructor (x: Double, y: Double, heading: Double) : this(Vec2(x, y), heading)

  val x: Double
    get() = pos.x

  val y: Double
    get() = pos.y

  override fun plus(other: Pose) = Pose(pos + other.pos, heading + other.heading)
  override fun minus(other: Pose) = Pose(pos - other.pos, heading - other.heading)
  override fun times(scalar: Double) = Pose(pos * scalar, heading * scalar)

  override fun toString(): String = "($x, $y, $heading)"

  companion object {
    val zero = Pose(0.0, 0.0, 0.0)
  }
}
