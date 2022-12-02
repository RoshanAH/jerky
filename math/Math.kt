package com.roshanah.jerky.math

import kotlin.math.abs
import kotlin.math.max
import kotlin.math.pow
import kotlin.math.sqrt

fun quadratic(a: Double, b: Double, c: Double) =
    listOf(
        (-b + sqrt(b.pow(2.0) - 4 * a * c)) / (2 * a),
        (-b - sqrt(b.pow(2.0) - 4 * a * c)) / (2 * a),
    )

// DO NOT USE THIS EXCESSIVELY as multiple layers of derivitives will take long to compute
val ((Double) -> Double).derivative: (Double) -> Double
  get() = {
    val point = this(it)
    val dx = max(abs(point) * 1e-14, 1e-14) // scaled to account for double rounding
    (this(it + dx * 0.5) - this(it - dx * 0.5)) / dx
  }

fun newtonMethodSolve(
    function: (Double) -> Double,
    initialGuess: Double,
    precision: Double = 1e-13,
): Double {
  var guess = initialGuess
  var error = -1.0

  while (error < 0 || error > precision) {
    val out = function(guess)
    // dx is scaled by the output of the function in order to compensate for double rounding
    val derivative = function.derivative(guess)
    val delta = -out / derivative

    guess += delta
    error = abs(out)
  }
  return guess
}