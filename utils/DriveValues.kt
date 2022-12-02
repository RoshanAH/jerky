package com.roshanah.jerky.utils

import com.roshanah.jerky.math.Linear

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

  companion object {
    val zero = DriveValues(0.0, 0.0, 0.0, 0.0)
  }
}
