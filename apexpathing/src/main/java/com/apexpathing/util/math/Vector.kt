package com.apexpathing.util.math

import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

/**
 * Mutable 2D vector with polar coordinate support.
 * Used by MecanumDrive for drive power calculations.
 */
data class Vector
    @JvmOverloads constructor(
        @get:JvmName("x") var x: Double = 0.0,
        @get:JvmName("y") var y: Double = 0.0
    )
{
    companion object {
        fun fromPolar(r: Double = 0.0, theta: Double = 0.0) =
            Vector(r * cos(theta), r * sin(theta))
    }

    @get:JvmName("magnitude")
    var r: Double
        get() = hypot(x, y)
        set(value) {
            val ang = theta
            x = value * cos(ang)
            y = value * sin(ang)
        }
    @get:JvmName("theta")
    var theta: Double
        get() = atan2(y, x)
        set(value) {
            val _r = r
            x = _r * cos(value)
            y = _r * sin(value)
        }

    fun xComponent() = x
    fun yComponent() = y

    fun dotProduct(otherVec: Vector) =
        this * otherVec
    fun crossProduct(otherVec: Vector) =
        ((x * otherVec.y) - (y * otherVec.x))
    fun rotateVec(ang: Double) {
        theta = normalize(theta + ang)
    }
    fun asPose() =
        Pose(x, y)

    operator fun plus(otherVec: Vector) =
        Vector(x + otherVec.x, y + otherVec.y)
    operator fun plus(otherPose: Pose) =
        Vector(x + otherPose.x, y + otherPose.y)
    operator fun minus(otherVec: Vector) =
        Vector(x - otherVec.x, y - otherVec.y)
    operator fun minus(otherPose: Pose) =
        Vector(x - otherPose.x, y - otherPose.y)
    operator fun times(otherVec: Vector) =
        ((x * otherVec.x) + (y * otherVec.y))
    operator fun times(scalar: Double) =
        Vector(x * scalar, y * scalar)
    operator fun div(scalar: Double) =
        Vector(x / scalar, y / scalar)
    operator fun unaryMinus() = (this * -1.0)

    override fun toString(): String = "<$x, $y>"
    fun debug(): String = "Vector <x: $x, y: $y>, <r: $r, θ: $theta>"
}