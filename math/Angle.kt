package com.roshanah.jerky.math

import kotlin.math.PI

class Angle internal constructor(radians: Double) : Linear<Angle>{
  // wrap angle from -pi to pi
    private val theta: Double = ((radians % (2 * PI)) + 3 * PI) % (2 * PI) - PI

    val rad: Double
        get() = theta
    val deg: Double
        get() = theta / PI * 180
    val grad: Double
        get() = theta / PI * 100
    val rev: Double
        get() = theta / (2 * PI)

    val dir: Vec2
        get() = Vec2.polar(1.0, this)

    override operator fun plus(other: Angle) = Angle(theta + other.theta)
    override operator fun unaryMinus() = Angle(-theta)
    override operator fun minus(other: Angle) = Angle(theta - other.theta)
    override operator fun times(scalar: Double): Angle = Angle(theta * scalar)

    override fun toString() = "${theta.toString()} rad"
}

val Double.rad : Angle
    get() = Angle(this)

val Double.deg : Angle
    get() = Angle(this / 180.0 * PI)

val Double.grad : Angle
    get() = Angle(this / 100.0 * PI)

val Double.rev: Angle
  get() = Angle(this * 2 * PI)

fun cos(angle: Angle) = kotlin.math.cos(angle.rad)
fun sin(angle: Angle) = kotlin.math.sin(angle.rad)
fun tan(angle: Angle) = kotlin.math.tan(angle.rad)
