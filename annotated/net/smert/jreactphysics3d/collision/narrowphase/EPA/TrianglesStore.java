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
package net.smert.jreactphysics3d.collision.narrowphase.EPA;

import units.qual.Dimensionless;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This class stores several triangles of the polytope in the EPA algorithm.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class TrianglesStore {

    private static final @Dimensionless int MAX_TRIANGLES = ((@Dimensionless int) (200));     // Maximum number of triangles

    // Number of triangles
    private @Dimensionless int numTriangles;

    // Triangles
    private final @Dimensionless TriangleEPA @Dimensionless [] triangles;

    // Constructor
    public TrianglesStore() {
        numTriangles = ((@Dimensionless int) (0));
        triangles = new @Dimensionless TriangleEPA @Dimensionless [MAX_TRIANGLES];
    }

    // Clear all the storage
    public void clear(@Dimensionless TrianglesStore this) {
        numTriangles = ((@Dimensionless int) (0));
    }

    // Return the number of triangles
    public int getNumTriangles(@Dimensionless TrianglesStore this) {
        return numTriangles;
    }

    public void setNumTriangles(@Dimensionless TrianglesStore this, int backup) {
        numTriangles = backup;
    }

    // Access operator
    public TriangleEPA get(@Dimensionless TrianglesStore this, int i) {
        assert (i >= ((@Dimensionless int) (0)) && i < triangles.length);
        return triangles[i];
    }

    // Return the last triangle
    public @Dimensionless TriangleEPA last(@Dimensionless TrianglesStore this) {
        assert (numTriangles > ((@Dimensionless int) (0)));
        return triangles[numTriangles - ((@Dimensionless int) (1))];
    }

    // Create a new triangle
    public TriangleEPA newTriangle(@Dimensionless TrianglesStore this, Vector3[] vertices, int v0, int v1, int v2) {
        @Dimensionless
        TriangleEPA newTriangle = null;

        // If we have not reached the maximum number of triangles
        if (numTriangles != MAX_TRIANGLES) {
            newTriangle = new @Dimensionless TriangleEPA(v0, v1, v2);
            triangles[numTriangles] = newTriangle;
            numTriangles++;
            if (!newTriangle.computeClosestPoint(vertices)) {
                numTriangles--;
                newTriangle = null;
                triangles[numTriangles] = null;
            }
        }

        // Return the new triangle
        return newTriangle;
    }

}
