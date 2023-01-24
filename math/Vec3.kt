package com.roshanah.jerky.math

import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

class Vec3(val x: Double, val y: Double, val z: Double) : Linear<Vec3>{

    val magnitude: Double
        get() = sqrt(x * x + y * y + z * z)
    val r: Double
        get() = sqrt(x * x + y * y)
    val theta: Angle
        get() = atan2(y, x).rad
    val phi: Angle
        get() = atan2(magnitude, z).rad
    val unit: Vec3
      get() = this / magnitude

    val xy: Vec2
      get() = Vec2(x, y)

    fun setMagnitude(r: Double) = normalize() * r

    fun add(other: Vec3) = Vec3(x + other.x, y + other.y, z + other.z)
    fun subtract(other: Vec3) = Vec3(x - other.x, y - other.y, z - other.z)
    fun translate(x: Double, y: Double) = add(Vec3(x, y, z))
    fun scale(scale: Double) = Vec3(x * scale, y * scale, z * scale)
    fun project(other: Vec3) = this * ((this dot other) / square) 

    fun rotateX(theta: Angle) = Mat3.rotationX(theta) * this
    fun rotateY(theta: Angle) = Mat3.rotationY(theta) * this
    fun rotateZ(theta: Angle) = Mat3.rotationZ(theta) * this

    fun normalize(): Vec3 {
      val mag = magnitude
      if (mag == 0.0) return zero
      return scale(1 / mag)
    }

    fun dist(other: Vec2): Double {
        return Math.sqrt(Math.pow(x - other.x, 2.0) + Math.pow(y - other.y, 2.0))
    }

    infix fun dot(other: Vec3): Double = x * other.x + y * other.y + z * other.z

    val square: Double
      get() = this dot this

    override operator fun plus(other: Vec3) = add(other)
    override operator fun minus(other: Vec3) = subtract(other)
    override operator fun unaryMinus() = this * -1.0
    override operator fun times(scalar: Double) = scale(scalar)

    override fun equals(other: Any?) = other is Vec2 && (x == other.x && y == other.y)

    override fun toString(): String {
        return "($x, $y, $z)"
    }

    companion object {
      val zero = Vec3(0.0, 0.0, 0.0)
      val one = Vec3(1.0, 1.0, 1.0)
      fun spherical(p: Double, phi: Angle, theta: Angle) = Vec3(p * sin(phi) * cos(theta), p * sin(phi) * sin(theta), p * cos(phi))
      fun cylindrical(r: Double, theta: Angle, z: Double) = Vec3(r * cos(theta), r * sin(theta), z)
    }
}

