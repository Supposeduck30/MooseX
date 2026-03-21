package com.xpathing.main.localization

import com.xpathing.util.math.Pose
import com.xpathing.util.math.Vector
import java.lang.annotation.Inherited

@MustBeDocumented
@Inherited
@Target(AnnotationTarget.CLASS)
@Retention(AnnotationRetention.RUNTIME)
annotation class Pathing

@Target(AnnotationTarget.CLASS)
@Retention(AnnotationRetention.RUNTIME)
@Inherited
@MustBeDocumented
annotation class Localizer

abstract class LocalizerBase {
    protected var lastPosition: Pose = Pose(0.0, 0.0, 0.0)
    protected var lastVelocity: Vector = Vector(0.0, 0.0)

    var currentPosition: Pose = Pose(0.0, 0.0, 0.0)
    var currentVelocity: Vector = Vector(0.0, 0.0)
    var currentAcceleration: Vector = Vector(0.0, 0.0)

    abstract fun initLocalizer(deviceName : String)
    abstract fun update()
    abstract fun setPose(pose: Pose)

    protected fun updateKinematics() {
        currentVelocity = Vector(
            currentPosition.x - lastPosition.x,
            currentPosition.y - lastPosition.y
        )
        currentAcceleration = currentVelocity.subtract(lastVelocity)
        lastVelocity = currentVelocity.copy()
        lastPosition = currentPosition.copyPose()
    }
}
