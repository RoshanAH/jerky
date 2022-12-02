package com.roshanah.jerky.math

// we want everything that implements linear to be added and subtracted from each other, as well as scaled
interface Linear<T>{
  operator fun plus(other: T): T
  operator fun minus(other: T): T
  operator fun times(scalar: Double): T
  operator fun div(scalar: Double) = times(1 / scalar)
  operator fun unaryMinus() = this * -1.0
}
