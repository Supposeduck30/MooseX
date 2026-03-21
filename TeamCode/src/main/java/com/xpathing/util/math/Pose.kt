package com.xpathing.util.math

import kotlin.math.PI
import kotlin.math.sqrt

data class Pose(
    @get:JvmName("x") var x: Double = 0.0,
    @get:JvmName("y") var y: Double = 0.0,
    @get:JvmName("heading") var heading: Double = 0.0
) {
    /**
     * Calculates the distance to another pose
     * Exactly the same as distanceFrom() however is included for user flexibility
     *
     * @return the distance to another pose
     */
    infix fun distanceTo(otherPose: Pose): Double = sqrt(distanceSquaredFrom(otherPose))

    /**
     * Calculates the distance from another pose to this pose
     * Exactly the same as distanceTo() however is included for user flexibility
     *
     * @return the distance from another pose
     */
    infix fun distanceFrom(otherPose: Pose): Double = distanceTo(otherPose)

    /**
     * Calculates the squared distance between one pose and another pose
     *
     * @return the squared distance to another pose
     */
    fun distanceSquaredFrom(otherPose: Pose): Double {
        val deltaX = otherPose.x - this.x
        val deltaY = otherPose.y - this.y
        return deltaX * deltaX + deltaY * deltaY
    }

    /**
     * Converts the pose to a vector
     *
     * @return the vector form of the current pose
     */
    fun asVector(): Vector = Vector(x, y)

    /**
     * Adds this pose and another pose together
     *
     * @param otherPose the pose to be added
     * @return the resulting sum as a pose
     */
    operator fun plus(otherPose: Pose): Pose {
        return Pose(
            x + otherPose.x,
            y + otherPose.y,
            heading + otherPose.heading
        )
    }

    /**
     * Subtracts another pose from this pose
     *
     * @param otherPose the pose to be subtracted
     * @return the resulting difference as a pose
     */
    operator fun minus(otherPose: Pose): Pose {
        return Pose(
            x - otherPose.x,
            y - otherPose.y,
            heading - otherPose.heading
        )
    }

    /**
     * Multiplies the x-y values by a certain amount
     *
     * @param scalar the scale factor to multiply by
     * @return the scaled pose
     */
    operator fun times(scalar: Double) = apply {
        x *= scalar
        y *= scalar
    }

    /**
     * Divides the x-y values by a certain amount
     *
     * @param scalar the scale factor to divide by
     * @return the scaled pose
     */
    operator fun div(scalar: Double) = apply {
        x /= scalar
        y /= scalar
    }

    fun copyPose(): Pose {
        return Pose(x, y, heading)
    }

    fun normalizeAngle(angle: Double): Double {
        var a = angle % (2 * PI)
        if (a > PI) a -= 2 * PI
        if (a <= -PI) a += 2 * PI
        return a
    }
}
