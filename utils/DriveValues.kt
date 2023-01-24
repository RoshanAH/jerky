package com.roshanah.jerky.utils

import com.roshanah.jerky.math.Linear
import com.roshanah.jerky.math.Pose
import com.roshanah.jerky.math.sin
import com.roshanah.jerky.math.cos
import com.roshanah.jerky.math.Mat2
import com.roshanah.jerky.profiling.Derivatives
import com.roshanah.jerky.utils.DriveValues

import kotlin.math.sign

data class DriveValues(
  val fl: Double,
  val fr: Double,
  val bl: Double,
  val br: Double,
) : Linear<DriveValues> {

  val sign: DriveValues
    get() = DriveValues(sign(fl), sign(fr), sign(bl), sign(br))

  override fun plus(other: DriveValues) = DriveValues(
    fl + other.fl,
    fr + other.fr,
    bl + other.bl,
    br + other.br,
  )

  override fun minus(other: DriveValues) = DriveValues(
    fl - other.fl,
    fr - other.fr,
    bl - other.bl,
    br - other.br,
  )

  override fun times(scalar: Double) = DriveValues(
    fl * scalar,
    fr * scalar,
    bl * scalar,
    br * scalar,
  )

  override fun toString(): String = "($fl, $fr, $bl, $br)"

  operator fun times(other: DriveValues) = DriveValues(fl * other.fl, fr * other.fr, bl * other.bl, br * other.br)

  companion object {
    val zero = DriveValues(0.0, 0.0, 0.0, 0.0)
    val forward = DriveValues(1.0, 1.0, 1.0, 1.0)
    val strafe = DriveValues(1.0, -1.0, -1.0, 1.0)
    val turn = DriveValues(-1.0, 1.0, -1.0, 1.0)
  }
}

data class DriveComponents (
  val forward: DriveValues,
  val strafe: DriveValues,
  val turn: DriveValues,
) : Linear<DriveComponents> {

  override fun plus(other: DriveComponents) = DriveComponents(
    forward + other.forward,
    strafe + other.strafe,
    turn + other.turn,
  )

  override fun minus(other: DriveComponents) = DriveComponents(
    forward + other.forward,
    strafe + other.strafe,
    turn + other.turn,
  )

  override fun times(scalar: Double) = DriveComponents(
    forward * scalar,
    strafe * scalar,
    turn * scalar
  )

  fun powers(kForward: Double, kStrafe: Double, kTurn: Double) = forward * kForward + strafe * kStrafe + turn * kTurn
  fun combine() = powers(1.0, 1.0, 1.0)

  companion object {
    val zero = DriveComponents(DriveValues.zero, DriveValues.zero, DriveValues.zero)
  }
}
