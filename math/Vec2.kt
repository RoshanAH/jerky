package com.roshanah.jerky.math

import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

class Vec2(val x: Double, val y: Double) : Linear<Vec2>{

    val magnitude: Double
        get() = Math.sqrt(x * x + y * y)
    val theta: Angle
        get() = atan2(y, x).rad
    val unit: Vec2
      get() = this / magnitude

    fun setMagnitude(r: Double) = normalize() * r

    fun setTheta(theta: Angle) = polar(magnitude, theta)

    fun rotate(theta: Angle) = Vec2(
      cos(theta) * x - sin(theta) * y,
      sin(theta) * x + cos(theta) * y
    )

    fun add(other: Vec2) = Vec2(x + other.x, y + other.y)
    fun subtract(other: Vec2) = Vec2(x - other.x, y - other.y)
    fun translate(x: Double, y: Double) = add(Vec2(x, y))
    fun scale(scale: Double) = Vec2(x * scale, y * scale)
    fun project(other: Vec2) = this * ((this dot other) / square) 

    fun dotInverse(): Vec2 {
        return Vec2(1 / (2 * x), 1 / (2 * y))
    }

    fun normalize(): Vec2 {
      val mag = magnitude
      if (mag == 0.0) return zero
      return scale(1 / mag)
    }

    fun dist(other: Vec2): Double {
        return Math.sqrt(Math.pow(x - other.x, 2.0) + Math.pow(y - other.y, 2.0))
    }

    infix fun dot(other: Vec2): Double = x * other.x + y * other.y

    val square: Double
      get() = this dot this

    override operator fun plus(other: Vec2) = add(other)
    override operator fun minus(other: Vec2) = subtract(other)

    override operator fun unaryMinus() = this * -1.0
    override operator fun times(scalar: Double) = scale(scalar)

    override fun equals(other: Any?) = other is Vec2 && (x == other.x && y == other.y)

    override fun toString(): String {
        return "($x, $y)"
    }

    companion object {
      val zero = Vec2(0.0, 0.0)
      val one = Vec2(1.0, 1.0)
      fun polar(r: Double, theta: Angle) = Vec2(r * cos(theta), r * sin(theta))
    }
}

