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
 * This class represents a triangle face of the current polytope in the EPA algorithm.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class TriangleEPA {

    // True if the triangle face is visible from the new support point
    private @Dimensionless boolean isObsolete;

    // Determinant
    private @Dimensionless float determinant;

    // Square distance of the point closest point v to the origin
    private @Dimensionless float distanceSquare;

    // Lambda1 value such that v = lambda0 * y_0 + lambda1 * y_1 + lambda2 * y_2
    private @Dimensionless float lambda1;

    // Lambda1 value such that v = lambda0 * y_0 + lambda1 * y_1 + lambda2 * y_2
    private @Dimensionless float lambda2;

    // Indices of the vertices y_i of the triangle
    private final @Dimensionless int @Dimensionless [] indicesVertices = new @Dimensionless int @Dimensionless [((@Dimensionless int) (3))];

    // Three adjacent edges of the triangle (edges of other triangles)
    private final @Dimensionless EdgeEPA @Dimensionless [] adjacentEdges = new @Dimensionless EdgeEPA @Dimensionless [((@Dimensionless int) (3))];

    // Point v closest to the origin on the affine hull of the triangle
    private final @Dimensionless Vector3 closestPoint;

    // Constructor
    public TriangleEPA() {
        this(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
    }

    // Constructor
    public TriangleEPA(@Dimensionless int indexVertex1, @Dimensionless int indexVertex2, @Dimensionless int indexVertex3) {
        isObsolete = false;
        determinant = ((@Dimensionless float) (0.0f));
        distanceSquare = ((@Dimensionless float) (0.0f));
        lambda1 = ((@Dimensionless float) (0.0f));
        lambda2 = ((@Dimensionless float) (0.0f));
        indicesVertices[((@Dimensionless int) (0))] = indexVertex1;
        indicesVertices[((@Dimensionless int) (1))] = indexVertex2;
        indicesVertices[((@Dimensionless int) (2))] = indexVertex3;
        closestPoint = new @Dimensionless Vector3();
    }

    // Compute the point v closest to the origin of this triangle
    public @Dimensionless boolean computeClosestPoint(@Dimensionless TriangleEPA this, @Dimensionless Vector3 @Dimensionless [] vertices) {

        @Dimensionless
        Vector3 p0 = new @Dimensionless Vector3(vertices[indicesVertices[((@Dimensionless int) (0))]]);
        @Dimensionless
        Vector3 p1 = new @Dimensionless Vector3(vertices[indicesVertices[((@Dimensionless int) (1))]]).subtract(p0);
        @Dimensionless
        Vector3 p2 = new @Dimensionless Vector3(vertices[indicesVertices[((@Dimensionless int) (2))]]).subtract(p0);

        @Dimensionless
        float p0DotP1 = p0.dot(p1);
        @Dimensionless
        float p0DotP2 = p0.dot(p2);
        @Dimensionless
        float p1DotP1 = p1.dot(p1);
        @Dimensionless
        float p1DotP2 = p1.dot(p2);
        @Dimensionless
        float p2DotP2 = p2.dot(p2);

        // Compute determinant
        determinant = p1DotP1 * p2DotP2 - p1DotP2 * p1DotP2;

        // Compute lambda values
        lambda1 = p0DotP2 * p1DotP2 - p0DotP1 * p2DotP2;
        lambda2 = p0DotP1 * p1DotP2 - p0DotP2 * p1DotP1;

        // If the determinant is positive
        if (determinant > ((@Dimensionless float) (0.0f))) {
            // Compute the closest point v
            p1.multiply(lambda1).add(p2.multiply(lambda2)).multiply(((@Dimensionless float) (1.0f)) / determinant);
            closestPoint.set(p0).add(p1);

            // Compute the square distance of closest point to the origin
            distanceSquare = closestPoint.dot(closestPoint);

            return true;
        }

        return false;
    }

    // Compute the point of an object closest to the origin
    public @Dimensionless Vector3 computeClosestPointOfObject(@Dimensionless TriangleEPA this, @Dimensionless Vector3 @Dimensionless [] supportPointsOfObject) {
        @Dimensionless
        Vector3 p0 = new @Dimensionless Vector3(supportPointsOfObject[indicesVertices[((@Dimensionless int) (0))]]);
        @Dimensionless
        Vector3 p1 = new @Dimensionless Vector3(supportPointsOfObject[indicesVertices[((@Dimensionless int) (1))]]).subtract(p0).multiply(lambda1);
        @Dimensionless
        Vector3 p2 = new @Dimensionless Vector3(supportPointsOfObject[indicesVertices[((@Dimensionless int) (2))]]).subtract(p0).multiply(lambda2);
        return p0.add(p1.add(p2).multiply(((@Dimensionless float) (1.0f)) / determinant));
    }

    // Execute the recursive silhouette algorithm from this triangle face.
    // The parameter "vertices" is an array that contains the vertices of the current polytope and the
    // parameter "indexNewVertex" is the index of the new vertex in this array. The goal of the
    // silhouette algorithm is to add the new vertex in the polytope by keeping it convex. Therefore,
    // the triangle faces that are visible from the new vertex must be removed from the polytope and we
    // need to add triangle faces where each face contains the new vertex and an edge of the silhouette.
    // The silhouette is the connected set of edges that are part of the border between faces that
    // are seen and faces that are not seen from the new vertex. This method starts from the nearest
    // face from the new vertex, computes the silhouette and create the new faces from the new vertex in
    // order that we always have a convex polytope. The faces visible from the new vertex are set
    // obselete and will not be considered as being a candidate face in the future.
    public @Dimensionless boolean computeSilhouette(@Dimensionless TriangleEPA this, @Dimensionless Vector3 @Dimensionless [] vertices, @Dimensionless int indexNewVertex, @Dimensionless TrianglesStore triangleStore) {

        @Dimensionless
        int first = triangleStore.getNumTriangles();

        // Mark the current triangle as obsolete because it
        setIsObsolete(true);

        // Execute recursively the silhouette algorithm for the adjacent edges of neighboring
        // triangles of the current triangle
        @Dimensionless
        boolean result = adjacentEdges[((@Dimensionless int) (0))].computeSilhouette(vertices, indexNewVertex, triangleStore)
                && adjacentEdges[((@Dimensionless int) (1))].computeSilhouette(vertices, indexNewVertex, triangleStore)
                && adjacentEdges[((@Dimensionless int) (2))].computeSilhouette(vertices, indexNewVertex, triangleStore);

        if (result) {

            // For each triangle face that contains the new vertex and an edge of the silhouette
            for (int i = first, j = triangleStore.getNumTriangles() - ((@Dimensionless int) (1)); i != triangleStore.getNumTriangles(); j = i++) {
                @Dimensionless
                TriangleEPA triangle = triangleStore.get(i);
                Utils.halfLink(triangle.getAdjacentEdge(((@Dimensionless int) (1))), new @Dimensionless EdgeEPA(triangle, ((@Dimensionless int) (1))));

                if (!Utils.link(new @Dimensionless EdgeEPA(triangle, ((@Dimensionless int) (0))), new @Dimensionless EdgeEPA(triangleStore.get(j), ((@Dimensionless int) (2))))) {
                    return false;
                }
            }

        }

        return result;
    }

    // Return true if the triangle face is obsolete
    public boolean getIsObsolete(@Dimensionless TriangleEPA this) {
        return isObsolete;
    }

    // Set the isObsolete value
    public void setIsObsolete(@Dimensionless TriangleEPA this, boolean isObsolete) {
        this.isObsolete = isObsolete;
    }

    // Return the square distance of the closest point to origin
    public @Dimensionless float getDistanceSquare(@Dimensionless TriangleEPA this) {
        return distanceSquare;
    }

    // Access operator
    public int getIndexVertex(@Dimensionless TriangleEPA this, int index) {
        assert (index >= ((@Dimensionless int) (0)) && index < ((@Dimensionless int) (3)));
        return indicesVertices[index];
    }

    // Return an edge of the triangle
    public EdgeEPA getAdjacentEdge(@Dimensionless TriangleEPA this, int index) {
        assert (index >= ((@Dimensionless int) (0)) && index < ((@Dimensionless int) (3)));
        return adjacentEdges[index];
    }

    // Set an adjacent edge of the triangle
    public void setAdjacentEdge(@Dimensionless TriangleEPA this, int index, EdgeEPA edge) {
        assert (index >= ((@Dimensionless int) (0)) && index < ((@Dimensionless int) (3)));
        adjacentEdges[index] = edge;
    }

    // Return the point closest to the origin
    public @Dimensionless Vector3 getClosestPoint(@Dimensionless TriangleEPA this) {
        return closestPoint;
    }

    // Return true if the closest point on affine hull is inside the triangle
    public @Dimensionless boolean isClosestPointInternalToTriangle(@Dimensionless TriangleEPA this) {
        return (lambda1 >= ((@Dimensionless float) (0.0f)) && lambda2 >= ((@Dimensionless float) (0.0f)) && (lambda1 + lambda2) <= determinant);
    }

    // Return true if the triangle is visible from a given vertex
    public boolean isVisibleFromVertex(@Dimensionless TriangleEPA this, Vector3[] vertices, int index) {
        @Dimensionless
        Vector3 closestToVert = new @Dimensionless Vector3(vertices[index]).subtract(closestPoint);
        return (closestPoint.dot(closestToVert) > ((@Dimensionless float) (0.0f)));
    }

}
