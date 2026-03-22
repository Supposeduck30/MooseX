@file:Suppress("unused")

package com.apexpathing.util.math

import kotlin.math.PI

/**
 * A coordinate system interface, meant to store data regarding certain coordinate systems
 * Current coordinate systems include:
 *  - ApexCoordinates
 *  - PedroCoordinates
 *
 * This library will use Apex Coordinates as the default system unless specified otherwise
 *
 * @author Achintya Akula - 30099 OmicronX
 */
interface CoordinateSystem {

    /**
     * Stores info on how to convert from a certain coordinate system to Apex coordinates
     *
     * @param pose The position to convert from a certain coordinate system into Apex coordinates
     * @return The converted pose in Apex Coordinates
     */
    fun toApexCoordinates(pose: Pose): Pose

    /**
     * Stores info on how to convert to a certain coordinate system from Apex coordinates
     *
     * @param pose The position to convert from apex coordinates into a certain coordinate system
     * @return The converted pose in said coordinate system
     */
    fun fromApexCoordinates(pose: Pose): Pose

    /**
     * Returns name of the coordinate system in String format for debugging
     *
     * @return name of coordinate system
     */
    fun name(): String
}

/**
 * A coordinate system identical to the FTC standard coordinate system
 * This is the default coordinate system of ApexPathing
 *
 * Origin(0,0) is situated at the center of the field
 *
 * x-axis runs from farthest field wall(-) -> closest field wall to audience(+)
 * y-axis runs perpendicular to the x-axis from left wall(-) to right wall(+) from the top view
 *
 * @author Achintya Akula - 30099 OmicronX
 */
object ApexCoordinates : CoordinateSystem {

    /**
     * Function to convert from Apex coordinates to Apex coordinates
     *
     * @param pose the position in Apex coordinates
     * @return the position in Apex coordinates
     */
    override fun toApexCoordinates(pose: Pose): Pose = pose

    /**
     * Function to convert to Apex coordinates from Apex coordinates
     *
     * @param pose the position in Apex coordinates
     * @return the position in Apex coordinates
     */
    override fun fromApexCoordinates(pose: Pose): Pose = pose

    /**
     * Returns name of the coordinate system in String format for debugging
     *
     * @return name of coordinate system -> "ApexCoordinates"
     */
    override fun name(): String = "ApexCoordinates"
}

/**
 * A coordinate System used by the PedroPathing library
 *
 * Origin(0,0) is situated at the bottom left corner of the field from the top view with the
 * audience at the bottom of the field
 *
 * x-axis runs from left(-) -> right(+) when viewed from the top
 * y-axis runs from audience-side(-) -> opposite field wall(+) when viewed from top
 *
 * This option is provided for users who are comfortable with PedroPathing, but isn't recommended
 *
 * @author Achintya Akula - 30099 OmicronX
 */
object PedroCoordinates : CoordinateSystem {

    /**
     * Function to convert from Pedro coordinates to Apex coordinates
     *
     * @param pose the position in Pedro coordinates
     * @return the position in Apex coordinates
     */
    override fun toApexCoordinates(pose: Pose): Pose {
        val rawPose = (pose.rotated(PI / 2) + Pose(72.0, 72.0))
        return Pose(rawPose.x, rawPose.y, rawPose.heading, ApexCoordinates)
    }

    /**
     * Function to convert to Pedro coordinates from Apex coordinates
     *
     * @param pose the position in Apex coordinates
     * @return the position in Pedro coordinates
     */
    override fun fromApexCoordinates(pose: Pose): Pose {
        val rawPose = (pose - Pose(72.0, 72.0)).rotated(-PI / 2)
        return Pose(rawPose.x, rawPose.y, rawPose.heading, this)
    }

    /**
     * Returns name of the coordinate system in String format for debugging
     *
     * @return name of coordinate system -> "PedroCoordinates"
     */
    override fun name(): String = "PedroCoordinates"
}
