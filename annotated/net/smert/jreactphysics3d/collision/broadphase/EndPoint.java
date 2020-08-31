/*
 * ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/
 * Copyright (c) 2010-2013 Daniel Chappuis
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from the
 * use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not claim
 *    that you wrote the original software. If you use this software in a
 *    product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 *
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 *
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * This file has been modified during the port to Java and differ from the source versions.
 */
package net.smert.jreactphysics3d.collision.broadphase;
import units.qual.Dimensionless;

/**
 * EndPoint structure that represent an end-point of an AABB on one of the three x,y or z axis.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
@Dimensionless
public class EndPoint {

    // ID of the AABB box corresponding to this end-point
    public @Dimensionless int boxID;

    // True if the end-point is a minimum end-point of a box
    public @Dimensionless boolean isMin;

    // Value (one dimension coordinate) of the end-point
    public @Dimensionless long value;

    // Set the values of the endpoint
    public void setValues(@Dimensionless EndPoint this, @Dimensionless int boxID, @Dimensionless boolean isMin, @Dimensionless long value) {
        this.boxID = boxID;
        this.isMin = isMin;
        this.value = value;
    }

}
