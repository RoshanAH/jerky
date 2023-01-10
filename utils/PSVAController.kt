package com.roshanah.jerky.utils

import com.roshanah.jerky.utils.DriveConstants
import com.roshanah.jerky.utils.DriveValues
import com.roshanah.jerky.math.Pose
import com.roshanah.jerky.math.rotation
import com.roshanah.jerky.math.Vec2
import com.roshanah.jerky.profiling.Derivatives
import kotlin.math.abs
import kotlin.math.sqrt
import kotlin.math.sign

class PSVAController(var constants: DriveConstants, vi: Pose = Pose.zero) {
  var lastTime = System.nanoTime() * 1e-9
  var vel = vi
    private set
  var motion = Derivatives(Pose.zero, Pose.zero, Pose.zero)
    private set

  fun update(tVel: Pose){
    val time = System.nanoTime() * 1e-9
    val deltaTime = time - lastTime
    lastTime = time
    
    if (tVel == Pose.zero){
      vel = Pose.zero
      Derivatives(Pose.zero, Pose.zero, Pose.zero)
      return
    }
  

    val difference = tVel - vel
    val r = constants.trackRadius

    val alpha: Double
    val a: Double

    if (difference.heading != 0.0) {
      val relation = (difference.pos).magnitude / abs(difference.heading)
      alpha = constants.maxAcceleration / (relation * sqrt(2.0) + 2 * r)
      a = relation * alpha
    } else if (difference.pos.magnitude != 0.0){
      alpha = 0.0
      a = constants.maxAcceleration / sqrt(2.0)
    } else{
      alpha = 0.0 
      a = 0.0
    }

    val accel = Pose(if (a == 0.0) Vec2.zero else difference.pos.unit * a, alpha * sign(difference.heading))
    val dv = accel * deltaTime

    vel = Pose(
      if (dv.pos.magnitude > difference.pos.magnitude)
        tVel.pos
      else 
        vel.pos + dv.pos,
      if (abs(dv.heading) > abs(difference.heading))
        tVel.heading
      else 
        vel.heading + dv.heading
    )

    motion = 
        Derivatives(
          Pose.zero,
          vel,
          accel
        )
  }
}
