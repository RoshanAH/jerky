package com.roshanah.jerky.math

class Mat2(val x1: Double, val y1: Double, val x2: Double, val y2: Double) : Linear<Mat2> {

  val det: Double
    get() = x1 * y2 - x2 * y1

  val inv: Mat2
    get() = Mat2(y2, -y1, -x2, x1) / det

  override fun plus(other: Mat2) =
      Mat2(
          x1 + other.x1,
          y1 + other.y1,
          x2 + other.x2,
          y2 + other.y2,
      )

  override fun minus(other: Mat2) =
      Mat2(
          x1 - other.x1,
          y1 - other.y1,
          x2 - other.x2,
          y2 - other.y2,
      )

  override fun times(scalar: Double) =
      Mat2(
          x1 * scalar,
          y1 * scalar,
          x2 * scalar,
          y2 * scalar,
      )

  operator fun times(other: Mat2) =
      Mat2(
          other.x1 * x1 + other.y1 * x2,
          other.x1 * y1 + other.y1 * y2,
          other.x2 * x1 + other.y2 * x2,
          other.x2 * y1 + other.y2 * y2,
      )

  operator fun times(other: Vec2) =
      Vec2(other.x * x1 + other.y * x2, other.x * y1 + other.y * y2)

  companion object{
    val identity: Mat2
      get() = Mat2(1.0, 0.0, 0.0, 1.0)

    fun rotation(theta: Angle) = 
        Mat2(
           cos(theta),
           sin(theta),
          -sin(theta),
           cos(theta)
        )
  }
}

