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

// DO NOT USE THIS EXCESSIVELY as multiple layers of derivatives will take long to compute
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

fun newtonMethodSolve(
    function: (Double) -> Double,
    lowerBound: Double,
    upperBound: Double,
    initialGuess: Double = (lowerBound + upperBound) * 0.5,
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
  return if (guess in lowerBound..upperBound) guess else throw IllegalArgumentException("newtons method did not converge")
}

fun bisectionMethodSolve(
    function: (Double) -> Double,
    lowerBound: Double,
    upperBound: Double,
    precision: Double = 1e-13,
): Double{
  val guess = (lowerBound + upperBound) * 0.5
  val error = function(guess)
  // println(guess)
  return if(abs(error) <= precision) guess
    // else if(upperBound - lowerBound < precision * 0.25) throw IllegalArgumentException("bisection method did not converge")
    else if(error < 0.0) bisectionMethodSolve(function, guess, upperBound, precision)
    else bisectionMethodSolve(function, lowerBound, guess, precision)
}
