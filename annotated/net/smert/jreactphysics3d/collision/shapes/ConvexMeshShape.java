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
package net.smert.jreactphysics3d.collision.shapes;

import units.qual.Dimensionless;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import net.smert.jreactphysics3d.configuration.Defaults;
import net.smert.jreactphysics3d.mathematics.Matrix3x3;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This class represents a convex mesh shape. In order to create a convex mesh shape, you need to indicate the
 * local-space position of the mesh vertices. You do it either by passing a vertices array to the constructor or using
 * the addVertex() method. Make sure that the set of vertices that you use to create the shape are indeed part of a
 * convex mesh. The center of mass of the shape will be at the origin of the local-space geometry that you use to create
 * the mesh. The method used for collision detection with a convex mesh shape has an O(n) running time with "n" beeing
 * the number of vertices in the mesh. Therefore, you should try not to use too many vertices. However, it is possible
 * to speed up the collision detection by using the edges information of your mesh. The running time of the collision
 * detection that uses the edges is almost O(1) constant time at the cost of additional memory used to store the
 * vertices. You can indicate edges information with the addEdge() method. Then, you must use the
 * setIsEdgesInformationUsed(true) method in order to use the edges information for collision detection.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
@Dimensionless
public class ConvexMeshShape extends CollisionShape {

    // True if the shape contains the edges of the convex mesh in order to
    // make the collision detection faster
    private @Dimensionless boolean isEdgesInformationUsed;

    // Cached support vertex index (previous support vertex)
    private @Dimensionless int cachedSupportVertex;

    // Array with the vertices of the mesh
    private final @Dimensionless List<@Dimensionless Vector3> vertices;

    // Mesh maximum bounds in the three local x, y and z directions
    private final @Dimensionless Vector3 maxBounds;

    // Mesh minimum bounds in the three local x, y and z directions
    private final @Dimensionless Vector3 minBounds;

    // Adjacency list representing the edges of the mesh
    private final @Dimensionless Map<@Dimensionless Integer, @Dimensionless List<@Dimensionless Integer>> edgesAdjacencyList;

    // Constructor.
    // If you use this constructor, you will need to set the vertices manually one by one using the addVertex() method.
    public ConvexMeshShape(@Dimensionless float margin) {
        super(CollisionShapeType.CONVEX_MESH, margin);
        isEdgesInformationUsed = false;
        cachedSupportVertex = ((@Dimensionless int) (0));
        vertices = new @Dimensionless ArrayList<>();
        maxBounds = new @Dimensionless Vector3();
        minBounds = new @Dimensionless Vector3();
        edgesAdjacencyList = new @Dimensionless HashMap<>();
    }

    // Constructor to initialize with a array of 3D vertices.
    // This method creates an internal copy of the input vertices.
    public ConvexMeshShape(@Dimensionless float @Dimensionless [] arrayVertices, @Dimensionless int nbVertices, @Dimensionless int stride, @Dimensionless float margin) {
        super(CollisionShapeType.CONVEX_MESH, margin);

        assert (arrayVertices != null);
        assert (nbVertices > ((@Dimensionless int) (0)));
        assert (stride > ((@Dimensionless int) (0)));

        isEdgesInformationUsed = false;
        cachedSupportVertex = ((@Dimensionless int) (0));
        vertices = new @Dimensionless ArrayList<>();
        maxBounds = new @Dimensionless Vector3();
        minBounds = new @Dimensionless Vector3();
        edgesAdjacencyList = new @Dimensionless HashMap<>();

        // Copy all the vertices into the internal array
        for (@Dimensionless int i = ((@Dimensionless int) (0)); i < arrayVertices.length; i += stride) {
            vertices.add(new @Dimensionless Vector3(arrayVertices[i + ((@Dimensionless int) (0))], arrayVertices[i + ((@Dimensionless int) (1))], arrayVertices[i + ((@Dimensionless int) (2))]));
        }

        // Recalculate the bounds of the mesh
        recalculateBounds();
    }

    // Copy-constructor
    public ConvexMeshShape(@Dimensionless ConvexMeshShape shape) {
        super(shape);

        // TODO: Do real copying
        isEdgesInformationUsed = shape.isEdgesInformationUsed;
        cachedSupportVertex = shape.cachedSupportVertex;
        vertices = shape.vertices;
        maxBounds = shape.maxBounds;
        minBounds = shape.minBounds;
        edgesAdjacencyList = shape.edgesAdjacencyList;
    }

    // Recompute the bounds of the mesh
    private void recalculateBounds(@Dimensionless ConvexMeshShape this) {

        maxBounds.zero();
        minBounds.zero();

        for (@Dimensionless Vector3 mVertice : vertices) {
            if (mVertice.getX() > maxBounds.getX()) {
                maxBounds.setX(mVertice.getX());
            }
            if (mVertice.getX() < minBounds.getX()) {
                minBounds.setX(mVertice.getX());
            }
            if (mVertice.getY() > maxBounds.getY()) {
                maxBounds.setY(mVertice.getY());
            }
            if (mVertice.getY() < minBounds.getY()) {
                minBounds.setY(mVertice.getY());
            }
            if (mVertice.getZ() > maxBounds.getZ()) {
                maxBounds.setZ(mVertice.getZ());
            }
            if (mVertice.getZ() < minBounds.getZ()) {
                minBounds.setZ(mVertice.getZ());
            }
        }

        // Add the object margin to the bounds
        maxBounds.add(new @Dimensionless Vector3(margin, margin, margin));
        minBounds.subtract(new @Dimensionless Vector3(margin, margin, margin));
    }

    // Return true if the edges information is used to speed up the collision detection
    public @Dimensionless boolean isEdgesInformationUsed(@Dimensionless ConvexMeshShape this) {
        return isEdgesInformationUsed;
    }

    // Set the variable to know if the edges information is used to speed up the
    // collision detection
    public void setIsEdgesInformationUsed(@Dimensionless ConvexMeshShape this, @Dimensionless boolean isEdgesUsed) {
        isEdgesInformationUsed = isEdgesUsed;
    }

    // Add an edge into the convex mesh by specifying the two vertex indices of the edge.
    // Note that the vertex indices start at zero and need to correspond to the order of
    // the vertices in the vertices array in the constructor or the order of the calls
    // of the addVertex() methods that you use to add vertices into the convex mesh.
    public void addEdge(@Dimensionless ConvexMeshShape this, @Dimensionless int v1, @Dimensionless int v2) {

        assert (v1 >= ((@Dimensionless int) (0)));
        assert (v2 >= ((@Dimensionless int) (0)));

        // If the entry for vertex v1 does not exist in the adjacency list
        if (edgesAdjacencyList.containsKey(v1) == false) {
            edgesAdjacencyList.put(v1, new @Dimensionless ArrayList<@Dimensionless Integer>());
        }

        // If the entry for vertex v2 does not exist in the adjacency list
        if (edgesAdjacencyList.containsKey(v2) == false) {
            edgesAdjacencyList.put(v2, new @Dimensionless ArrayList<@Dimensionless Integer>());
        }

        // Add the edge in the adjacency list
        edgesAdjacencyList.get(v1).add(v2);
        edgesAdjacencyList.get(v2).add(v1);
    }

    // Add a vertex into the convex mesh
    public void addVertex(@Dimensionless ConvexMeshShape this, @Dimensionless Vector3 vertex) {

        // Add the vertex in to vertices array
        vertices.add(vertex);

        // Update the bounds of the mesh
        if (vertex.getX() > maxBounds.getX()) {
            maxBounds.setX(vertex.getX());
        }
        if (vertex.getX() < minBounds.getX()) {
            minBounds.setX(vertex.getX());
        }
        if (vertex.getY() > maxBounds.getY()) {
            maxBounds.setY(vertex.getY());
        }
        if (vertex.getY() < minBounds.getY()) {
            minBounds.setY(vertex.getY());
        }
        if (vertex.getZ() > maxBounds.getZ()) {
            maxBounds.setZ(vertex.getZ());
        }
        if (vertex.getZ() < minBounds.getZ()) {
            minBounds.setZ(vertex.getZ());
        }
    }

    // Test equality between two cone shapes
    @Override
    public @Dimensionless boolean isEqualTo(@Dimensionless ConvexMeshShape this, @Dimensionless CollisionShape otherCollisionShape) {
        @Dimensionless
        ConvexMeshShape otherShape = (@Dimensionless ConvexMeshShape) otherCollisionShape;

        if (vertices.size() != otherShape.vertices.size()) {
            return false;
        }

        // If edges information is used, it means that a collison shape object will store
        // cached data (previous support vertex) and therefore, we should not reuse the shape
        // for another body. Therefore, we consider that all convex mesh shape using edges
        // information are different.
        if (isEdgesInformationUsed) {
            return false;
        }

        if (edgesAdjacencyList.size() != otherShape.edgesAdjacencyList.size()) {
            return false;
        }

        // Check that the vertices are the same
        for (@Dimensionless int i = ((@Dimensionless int) (0)); i < vertices.size(); i++) {
            if (vertices.get(i) != otherShape.vertices.get(i)) {
                return false;
            }
        }

        // Check that the edges are the same
        for (@Dimensionless int i = ((@Dimensionless int) (0)); i < edgesAdjacencyList.size(); i++) {

            assert (otherShape.edgesAdjacencyList.containsKey(i) == true);
            if (edgesAdjacencyList.get(i) != otherShape.edgesAdjacencyList.get(i)) {
                return false;
            }
        }

        return true;
    }

    // Return a local support point in a given direction with the object margin
    @Override
    public @Dimensionless Vector3 getLocalSupportPointWithMargin(@Dimensionless ConvexMeshShape this, @Dimensionless Vector3 direction, @Dimensionless Vector3 supportPoint) {

        // Get the support point without the margin
        getLocalSupportPointWithoutMargin(direction, supportPoint);

        // Get the unit direction vector
        @Dimensionless
        Vector3 unitDirection = new @Dimensionless Vector3(((@Dimensionless float) (1.0f)), ((@Dimensionless float) (1.0f)), ((@Dimensionless float) (1.0f)));
        if (direction.lengthSquare() >= Defaults.MACHINE_EPSILON * Defaults.MACHINE_EPSILON) {
            unitDirection.set(direction);
        }
        unitDirection.normalize();

        // Add the margin to the support point and return it
        return supportPoint.add(unitDirection.multiply(margin));
    }

    // Return a local support point in a given direction without the object margin.
    // If the edges information is not used for collision detection, this method will go through
    // the whole vertices list and pick up the vertex with the largest dot product in the support
    // direction. This is an O(n) process with "n" being the number of vertices in the mesh.
    // However, if the edges information is used, we can cache the previous support vertex and use
    // it as a start in a hill-climbing (local search) process to find the new support vertex which
    // will be in most of the cases very close to the previous one. Using hill-climbing, this method
    // runs in almost constant time.
    @Override
    public @Dimensionless Vector3 getLocalSupportPointWithoutMargin(@Dimensionless ConvexMeshShape this, @Dimensionless Vector3 direction, @Dimensionless Vector3 supportPoint) {

        // If the edges information is used to speed up the collision detection
        if (isEdgesInformationUsed) {

            assert (edgesAdjacencyList.size() == vertices.size());

            @Dimensionless
            int maxVertex = cachedSupportVertex;
            @Dimensionless
            float maxDotProduct = direction.dot(vertices.get(maxVertex));
            @Dimensionless
            boolean isOptimal;

            // Perform hill-climbing (local search)
            do {
                isOptimal = true;

                assert (edgesAdjacencyList.get(maxVertex).size() > ((@Dimensionless int) (0)));

                // For all neighbors of the current vertex

                // Raw iterator causes unsat in inference because we don't generate a qualifier
                // variable for the type arg. Instead, it is assumed that the type argument is
                // @Top Object
                // fix: don't use raw iterator
                @Dimensionless
                Iterator<@Dimensionless Integer> it = edgesAdjacencyList.get(maxVertex).iterator();
                while (it.hasNext()) {
                    @Dimensionless
                    int i = (@Dimensionless int) it.next();    // here
                    // Compute the dot product
                    @Dimensionless
                    float dotProduct = direction.dot(vertices.get(i));  // here

                    // If the current vertex is a better vertex (larger dot product)
                    if (dotProduct > maxDotProduct) {
                        maxVertex = i;
                        maxDotProduct = dotProduct;
                        isOptimal = false;
                    }
                }

            } while (!isOptimal);

            // Cache the support vertex
            cachedSupportVertex = maxVertex;

            // Return the support vertex
            return supportPoint.set(vertices.get(maxVertex));
        }

        // If the edges information is not used
        @Dimensionless
        float maxDotProduct = Defaults.DECIMAL_SMALLEST;
        @Dimensionless
        int indexMaxDotProduct = ((@Dimensionless int) (0));

        // For each vertex of the mesh
        for (@Dimensionless int i = ((@Dimensionless int) (0)); i < vertices.size(); i++) {

            // Compute the dot product of the current vertex
            @Dimensionless
            float dotProduct = direction.dot(vertices.get(i));

            // If the current dot product is larger than the maximum one
            if (dotProduct > maxDotProduct) {
                indexMaxDotProduct = i;
                maxDotProduct = dotProduct;
            }
        }

        assert (maxDotProduct >= ((@Dimensionless float) (0.0f)));

        // Return the vertex with the largest dot product in the support direction
        return supportPoint.set(vertices.get(indexMaxDotProduct));
    }

    // Return the local inertia tensor of the collision shape.
    // The local inertia tensor of the convex mesh is approximated using the inertia tensor
    // of its bounding box.
    @Override
    public void computeLocalInertiaTensor(@Dimensionless ConvexMeshShape this, @Dimensionless Matrix3x3 tensor, @Dimensionless float mass) {
        @Dimensionless
        float factor = (((@Dimensionless float) (1.0f)) / ((@Dimensionless float) (3.0f))) * mass;
        @Dimensionless
        Vector3 realExtent = new @Dimensionless Vector3(maxBounds).subtract(minBounds).multiply(((@Dimensionless float) (0.5f)));
        assert (realExtent.getX() > ((@Dimensionless int) (0)) && realExtent.getY() > ((@Dimensionless int) (0)) && realExtent.getZ() > ((@Dimensionless int) (0)));
        @Dimensionless
        float xSquare = realExtent.getX() * realExtent.getX();
        @Dimensionless
        float ySquare = realExtent.getY() * realExtent.getY();
        @Dimensionless
        float zSquare = realExtent.getZ() * realExtent.getZ();
        tensor.set(factor * (ySquare + zSquare), ((@Dimensionless float) (0.0f)), ((@Dimensionless float) (0.0f)),
                ((@Dimensionless float) (0.0f)), factor * (xSquare + zSquare), ((@Dimensionless float) (0.0f)),
                ((@Dimensionless float) (0.0f)), ((@Dimensionless float) (0.0f)), factor * (xSquare + ySquare));
    }

    // Return the local bounds of the shape in x, y and z directions
    @Override
    public void getLocalBounds(@Dimensionless ConvexMeshShape this, @Dimensionless Vector3 min, @Dimensionless Vector3 max) {
        min.set(minBounds);
        max.set(maxBounds);
    }

    @Override
    public @Dimensionless CollisionShape clone(@Dimensionless ConvexMeshShape this) {
        return new @Dimensionless ConvexMeshShape(this);
    }

}
