package com.roshanah.jerky.math


// |x1 x2 x3|
// |y1 y2 y3|
// |z1 z2 z3|

class Mat3(
    val x1: Double, 
    val y1: Double, 
    val z1: Double, 
    val x2: Double, 
    val y2: Double, 
    val z2: Double,
    val x3: Double, 
    val y3: Double, 
    val z3: Double
) : Linear<Mat3> {

  val det: Double
    get() = x1 * (y2 * z3 - y3 * z2) - x2 * (y1 * z3 - y3 * z1) + x3 * (y1 * z2 - y2 * z1)

  val inv: Mat3
    get() = Mat3(
        (y2 * z3 - y3 * z2), -(y1 * z3 - y3 * z1),  (y1 * z2 - y2 * z1),
       -(x2 * z3 - x3 * z2),  (x1 * z3 - x3 * z1), -(x1 * z2 - x2 * z1),   
        (x2 * y3 - x3 * y2), -(x1 * y3 - x3 * y1),  (x1 * y2 - x2 * y1),
    ) / det

  val columns: List<Vec3>
    get() = listOf(
              Vec3(x1, y1, z1),
              Vec3(x2, y2, z2),
              Vec3(x3, y3, z3),
            )

  override fun plus(other: Mat3) =
      Mat3(
          x1 + other.x1,
          y1 + other.y1,
          z1 + other.z1,
          x2 + other.x2,
          y2 + other.y2,
          z2 + other.z2,
          x3 + other.x3,
          y3 + other.y3,
          z3 + other.z3,
      )

  override fun minus(other: Mat3) =
      Mat3(
          x1 - other.x1,
          y1 - other.y1,
          z1 - other.z1,
          x2 - other.x2,
          y2 - other.y2,
          z2 - other.z2,
          x3 - other.x3,
          y3 - other.y3,
          z3 - other.z3,
      )

  override fun times(scalar: Double) =
      Mat3(
          x1 * scalar,
          y1 * scalar,
          z1 * scalar,
          x2 * scalar,
          y2 * scalar,
          z2 * scalar,
          x3 * scalar,
          y3 * scalar,
          z3 * scalar,
      )

  operator fun times(other: Mat3) = 
      Mat3(
        other.x1 * x1 + other.y1 * x2 + other.z1 * x3,    
        other.x1 * y1 + other.y1 * y2 + other.z1 * y3,    
        other.x1 * z1 + other.y1 * z2 + other.z1 * z3,    
        other.x2 * x1 + other.y2 * x2 + other.z2 * x3,    
        other.x2 * y1 + other.y2 * y2 + other.z2 * y3,    
        other.x2 * z1 + other.y2 * z2 + other.z2 * z3,    
        other.x3 * x1 + other.y3 * x2 + other.z3 * x3,    
        other.x3 * y1 + other.y3 * y2 + other.z3 * y3,    
        other.x3 * z1 + other.y3 * z2 + other.z3 * z3,    
      )

  operator fun times(other: Vec3) =
      Vec3(
        other.x * x1 + other.y * x2 + other.z * x3,
        other.x * y1 + other.y * y2 + other.z * y3,
        other.x * z1 + other.y * z2 + other.z * z3,
      )

  companion object{
    val identity: Mat3
      get() = Mat3(
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0,
              )

    fun rotationX(theta: Angle) = 
        Mat3(
          1.0,  0.0,        0.0,
          0.0,  cos(theta), sin(theta),
          0.0, -sin(theta), cos(theta),
        )

    fun rotationY(theta: Angle) = 
        Mat3(
          cos(theta),  0.0, -sin(theta),
          0.0,         1.0,  0.0, 
          sin(theta),  0.0,  cos(theta),
        )

    fun rotationZ(theta: Angle) = 
        Mat3(
          cos(theta), sin(theta), 0.0,
         -sin(theta), cos(theta), 0.0,
          0.0,        0.0,        1.0,
        )
  } 
}

