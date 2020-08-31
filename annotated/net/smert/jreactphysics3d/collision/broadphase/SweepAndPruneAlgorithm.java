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
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import net.smert.jreactphysics3d.body.CollisionBody;
import net.smert.jreactphysics3d.collision.CollisionDetection;
import net.smert.jreactphysics3d.collision.shapes.AABB;

/**
 * This class implements the Sweep-And-Prune (SAP) broad-phase collision detection algorithm. This class implements an
 * array-based implementation of the algorithm from Pierre Terdiman that is described here :
 * www.codercorner.com/SAP.pdf.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
@Dimensionless
public class SweepAndPruneAlgorithm extends BroadPhaseAlgorithm {

    // Invalid array index
    protected final static @Dimensionless int INVALID_INDEX = ((@Dimensionless int) (Integer.MAX_VALUE));

    // Number of sentinel end-points in the array of a given axis
    protected final static @Dimensionless int NUM_SENTINELS = ((@Dimensionless int) (2));

    // Number of AABB boxes in the broad-phase
    protected @Dimensionless int numBoxes;

    // Max number of boxes in the boxes array
    protected @Dimensionless int numMaxBoxes;

    // Indices that are not used by any boxes
    protected final @Dimensionless ArrayList<@Dimensionless Integer> freeBoxIndices;

    // Array that contains all the AABB boxes of the broad-phase
    protected @Dimensionless BoxAABB @Dimensionless [] boxes;

    // Array of end-points on the three axis
    protected final @Dimensionless EndPoint @Dimensionless [] @Dimensionless [] endPoints = new @Dimensionless EndPoint @Dimensionless [] @Dimensionless [] {null, null, null};

    // Map a body pointer to a box index
    protected final @Dimensionless Map<@Dimensionless CollisionBody, @Dimensionless Integer> mapBodyToBoxIndex;

    // Constructor
    public SweepAndPruneAlgorithm(@Dimensionless CollisionDetection collisionDetection) {
        super(collisionDetection);
        numBoxes = ((@Dimensionless int) (0));
        numMaxBoxes = ((@Dimensionless int) (0));
        boxes = null;
        freeBoxIndices = new @Dimensionless ArrayList<>();
        mapBodyToBoxIndex = new @Dimensionless HashMap<>();
    }

    // Resize the boxes and end-points arrays when it is full
    protected void resizeArrays(@Dimensionless SweepAndPruneAlgorithm this) {

        // New number of boxes in the array
        @Dimensionless
        int newNumMaxBoxes = numMaxBoxes > ((@Dimensionless int) (0)) ? ((@Dimensionless int) (2)) * numMaxBoxes : ((@Dimensionless int) (100));
        @Dimensionless
        int numEndPoints = numBoxes * ((@Dimensionless int) (2)) + NUM_SENTINELS;
        @Dimensionless
        int newNumEndPoints = newNumMaxBoxes * ((@Dimensionless int) (2)) + NUM_SENTINELS;

        // Allocate memory for the new boxes and end-points arrays
        @Dimensionless
        BoxAABB @Dimensionless [] newBoxesArray = new @Dimensionless BoxAABB @Dimensionless [newNumMaxBoxes];
        @Dimensionless
        EndPoint @Dimensionless [] newEndPointsXArray = new @Dimensionless EndPoint @Dimensionless [newNumEndPoints];
        @Dimensionless
        EndPoint @Dimensionless [] newEndPointsYArray = new @Dimensionless EndPoint @Dimensionless [newNumEndPoints];
        @Dimensionless
        EndPoint @Dimensionless [] newEndPointsZArray = new @Dimensionless EndPoint @Dimensionless [newNumEndPoints];

        assert (newBoxesArray != null);
        assert (newEndPointsXArray != null);
        assert (newEndPointsYArray != null);
        assert (newEndPointsZArray != null);

        // If the arrays were not empty before
        if (numBoxes > ((@Dimensionless int) (0))) {

            // Copy the data from the old arrays into the new one
            System.arraycopy(boxes, ((@Dimensionless int) (0)), newBoxesArray, ((@Dimensionless int) (0)), numBoxes);
            System.arraycopy(endPoints[((@Dimensionless int) (0))], ((@Dimensionless int) (0)), newEndPointsXArray, ((@Dimensionless int) (0)), numEndPoints);
            System.arraycopy(endPoints[((@Dimensionless int) (1))], ((@Dimensionless int) (0)), newEndPointsYArray, ((@Dimensionless int) (0)), numEndPoints);
            System.arraycopy(endPoints[((@Dimensionless int) (2))], ((@Dimensionless int) (0)), newEndPointsZArray, ((@Dimensionless int) (0)), numEndPoints);
        } else {   // If the arrays were empty

            // Add the limits endpoints (sentinels) into the array
            @Dimensionless
            long min = Utils.encodeFloatIntoInteger(- ((@Dimensionless float) (Float.MAX_VALUE)));
            @Dimensionless
            long max = Utils.encodeFloatIntoInteger(((@Dimensionless float) (Float.MAX_VALUE)));
            newEndPointsXArray[((@Dimensionless int) (0))] = new @Dimensionless EndPoint();
            newEndPointsXArray[((@Dimensionless int) (0))].setValues(INVALID_INDEX, true, min);
            newEndPointsXArray[((@Dimensionless int) (1))] = new @Dimensionless EndPoint();
            newEndPointsXArray[((@Dimensionless int) (1))].setValues(INVALID_INDEX, false, max);
            newEndPointsYArray[((@Dimensionless int) (0))] = new @Dimensionless EndPoint();
            newEndPointsYArray[((@Dimensionless int) (0))].setValues(INVALID_INDEX, true, min);
            newEndPointsYArray[((@Dimensionless int) (1))] = new @Dimensionless EndPoint();
            newEndPointsYArray[((@Dimensionless int) (1))].setValues(INVALID_INDEX, false, max);
            newEndPointsZArray[((@Dimensionless int) (0))] = new @Dimensionless EndPoint();
            newEndPointsZArray[((@Dimensionless int) (0))].setValues(INVALID_INDEX, true, min);
            newEndPointsZArray[((@Dimensionless int) (1))] = new @Dimensionless EndPoint();
            newEndPointsZArray[((@Dimensionless int) (1))].setValues(INVALID_INDEX, false, max);
        }

        // Delete the old arrays
        // Assign the pointer to the new arrays
        boxes = newBoxesArray;
        endPoints[((@Dimensionless int) (0))] = newEndPointsXArray;
        endPoints[((@Dimensionless int) (1))] = newEndPointsYArray;
        endPoints[((@Dimensionless int) (2))] = newEndPointsZArray;

        numMaxBoxes = newNumMaxBoxes;
    }

    // Shrink the boxes and end-points arrays when too much memory is allocated
    protected void shrinkArrays(@Dimensionless SweepAndPruneAlgorithm this) {

        // New number of boxes and end-points in the array
        @Dimensionless
        int nextPowerOf2 = PairManager.ComputeNextPowerOfTwo((numBoxes - ((@Dimensionless int) (1))) / ((@Dimensionless int) (100)));
        @Dimensionless
        int newNumMaxBoxes = (numBoxes > ((@Dimensionless int) (100))) ? nextPowerOf2 * ((@Dimensionless int) (100)) : ((@Dimensionless int) (100));
        @Dimensionless
        int numEndPoints = numBoxes * ((@Dimensionless int) (2)) + NUM_SENTINELS;
        @Dimensionless
        int newNumEndPoints = newNumMaxBoxes * ((@Dimensionless int) (2)) + NUM_SENTINELS;

        assert (newNumMaxBoxes < numMaxBoxes);

        // Sort the list of the free boxes indices in ascending order
        Collections.sort(freeBoxIndices);

        // Reorganize the boxes inside the boxes array so that all the boxes are at the
        // beginning of the array
        @Dimensionless
        Map<@Dimensionless CollisionBody, @Dimensionless Integer> newMapBodyToBoxIndex = new @Dimensionless HashMap<>();
        for (Map.@Dimensionless Entry it : mapBodyToBoxIndex.entrySet()) {

            @Dimensionless
            CollisionBody body = (@Dimensionless CollisionBody) it.getKey();
            @Dimensionless
            int boxIndex = (@Dimensionless int) it.getValue();

            // If the box index is outside the range of the current number of boxes
            if (boxIndex >= numBoxes) {

                assert (!freeBoxIndices.isEmpty());

                // Get a new box index for that body (from the list of free box indices)
                @Dimensionless
                int newBoxIndex = freeBoxIndices.get(((@Dimensionless int) (0)));
                freeBoxIndices.remove(((@Dimensionless int) (0)));
                assert (newBoxIndex < numBoxes);

                // Copy the box to its new location in the boxes array
                @Dimensionless
                BoxAABB oldBox = boxes[boxIndex];
                @Dimensionless
                BoxAABB newBox = boxes[newBoxIndex];
                assert (oldBox.body.getBodyID() == body.getBodyID());
                newBox.body = oldBox.body;
                for (@Dimensionless int axis = ((@Dimensionless int) (0)); axis < ((@Dimensionless int) (3)); axis++) {

                    // Copy the minimum and maximum end-points indices
                    newBox.min[axis] = oldBox.min[axis];
                    newBox.max[axis] = oldBox.max[axis];

                    // Update the box index of the end-points
                    @Dimensionless
                    EndPoint minimumEndPoint = endPoints[axis][newBox.min[axis]];
                    @Dimensionless
                    EndPoint maximumEndPoint = endPoints[axis][newBox.max[axis]];
                    assert (minimumEndPoint.boxID == boxIndex);
                    assert (maximumEndPoint.boxID == boxIndex);
                    minimumEndPoint.boxID = newBoxIndex;
                    maximumEndPoint.boxID = newBoxIndex;
                }

                newMapBodyToBoxIndex.put(body, newBoxIndex);
            } else {
                newMapBodyToBoxIndex.put(body, boxIndex);
            }
        }

        assert (newMapBodyToBoxIndex.size() == mapBodyToBoxIndex.size());
        mapBodyToBoxIndex.clear();
        mapBodyToBoxIndex.putAll(newMapBodyToBoxIndex);

        // Allocate memory for the new boxes and end-points arrays
        @Dimensionless
        BoxAABB @Dimensionless [] newBoxesArray = new @Dimensionless BoxAABB @Dimensionless [newNumMaxBoxes];
        @Dimensionless
        EndPoint @Dimensionless [] newEndPointsXArray = new @Dimensionless EndPoint @Dimensionless [newNumEndPoints];
        @Dimensionless
        EndPoint @Dimensionless [] newEndPointsYArray = new @Dimensionless EndPoint @Dimensionless [newNumEndPoints];
        @Dimensionless
        EndPoint @Dimensionless [] newEndPointsZArray = new @Dimensionless EndPoint @Dimensionless [newNumEndPoints];

        assert (newBoxesArray != null);
        assert (newEndPointsXArray != null);
        assert (newEndPointsYArray != null);
        assert (newEndPointsZArray != null);

        // Copy the data from the old arrays into the new one
        System.arraycopy(boxes, ((@Dimensionless int) (0)), newBoxesArray, ((@Dimensionless int) (0)), numBoxes);
        System.arraycopy(endPoints[((@Dimensionless int) (0))], ((@Dimensionless int) (0)), newEndPointsXArray, ((@Dimensionless int) (0)), numEndPoints);
        System.arraycopy(endPoints[((@Dimensionless int) (1))], ((@Dimensionless int) (0)), newEndPointsYArray, ((@Dimensionless int) (0)), numEndPoints);
        System.arraycopy(endPoints[((@Dimensionless int) (2))], ((@Dimensionless int) (0)), newEndPointsZArray, ((@Dimensionless int) (0)), numEndPoints);

        // Delete the old arrays
        // Assign the pointer to the new arrays
        boxes = newBoxesArray;
        endPoints[((@Dimensionless int) (0))] = newEndPointsXArray;
        endPoints[((@Dimensionless int) (1))] = newEndPointsYArray;
        endPoints[((@Dimensionless int) (2))] = newEndPointsZArray;

        numMaxBoxes = newNumMaxBoxes;
    }

    // Check for 1D box intersection between two boxes that are sorted on the given axis.
    // Therefore, only one test is necessary here. We know that the
    // minimum of box1 cannot be larger that the maximum of box2 on the axis.
    protected @Dimensionless boolean testIntersect1DSortedAABBs(@Dimensionless SweepAndPruneAlgorithm this, @Dimensionless BoxAABB box1, @Dimensionless AABBInt box2, @Dimensionless EndPoint @Dimensionless [] endPointsArray, @Dimensionless int axis) {
        return !(endPointsArray[box1.max[axis]].value < box2.min[axis]);
    }

    // Check for 2D box intersection. This method is used when we know
    // that two boxes already overlap on one axis and when want to test
    // if they also overlap on the two others axis.
    protected @Dimensionless boolean testIntersect2D(@Dimensionless SweepAndPruneAlgorithm this, @Dimensionless BoxAABB box1, @Dimensionless BoxAABB box2, @Dimensionless int axis1, @Dimensionless int axis2) {
        return !(box2.max[axis1] < box1.min[axis1] || box1.max[axis1] < box2.min[axis1]
                || box2.max[axis2] < box1.min[axis2] || box1.max[axis2] < box2.min[axis2]);
    }

    // Notify the broad-phase that the AABB of an object has changed.
    // The input is an AABB with integer coordinates
    protected void updateObjectIntegerAABB(@Dimensionless SweepAndPruneAlgorithm this, @Dimensionless CollisionBody body, @Dimensionless AABBInt aabbInt) {

        assert (aabbInt.min[((@Dimensionless int) (0))] > Utils.encodeFloatIntoInteger(- ((@Dimensionless float) (Float.MAX_VALUE))));
        assert (aabbInt.min[((@Dimensionless int) (1))] > Utils.encodeFloatIntoInteger(- ((@Dimensionless float) (Float.MAX_VALUE))));
        assert (aabbInt.min[((@Dimensionless int) (2))] > Utils.encodeFloatIntoInteger(- ((@Dimensionless float) (Float.MAX_VALUE))));
        assert (aabbInt.max[((@Dimensionless int) (0))] < Utils.encodeFloatIntoInteger(((@Dimensionless float) (Float.MAX_VALUE))));
        assert (aabbInt.max[((@Dimensionless int) (1))] < Utils.encodeFloatIntoInteger(((@Dimensionless float) (Float.MAX_VALUE))));
        assert (aabbInt.max[((@Dimensionless int) (2))] < Utils.encodeFloatIntoInteger(((@Dimensionless float) (Float.MAX_VALUE))));

        // Get the corresponding box
        @Dimensionless
        int boxIndex = mapBodyToBoxIndex.get(body);
        @Dimensionless
        BoxAABB box = boxes[boxIndex];

        // Current axis
        for (@Dimensionless int axis = ((@Dimensionless int) (0)); axis < ((@Dimensionless int) (3)); axis++) {

            // Get the two others axis
            @Dimensionless
            int otherAxis1 = (((@Dimensionless int) (1)) << axis) & ((@Dimensionless int) (3));
            @Dimensionless
            int otherAxis2 = (((@Dimensionless int) (1)) << otherAxis1) & ((@Dimensionless int) (3));

            // Get the starting end-point of the current axis
            @Dimensionless
            EndPoint @Dimensionless [] startEndPointsCurrentAxis = endPoints[axis];

            // -------- Update the minimum end-point ------------//
            @Dimensionless
            EndPoint currentMinEndPoint = startEndPointsCurrentAxis[box.min[axis]];
            @Dimensionless
            int currentMinEndPointIndex = box.min[axis];
            assert (currentMinEndPoint.isMin);

            // Get the minimum value of the AABB on the current axis
            @Dimensionless
            long limit = aabbInt.min[axis];

            // If the minimum value of the AABB is smaller
            // than the current minimum endpoint
            if (limit < currentMinEndPoint.value) {

                currentMinEndPoint.value = limit;

                // The minimum end-point is moving left
                @Dimensionless
                EndPoint savedEndPoint = currentMinEndPoint;
                @Dimensionless
                int indexEndPoint = currentMinEndPointIndex;
                @Dimensionless
                int savedEndPointIndex = indexEndPoint;

                while ((currentMinEndPoint = startEndPointsCurrentAxis[--currentMinEndPointIndex]).value > limit) {
                    @Dimensionless
                    BoxAABB id1 = boxes[currentMinEndPoint.boxID];
                    @Dimensionless
                    boolean isMin = currentMinEndPoint.isMin;

                    // If it's a maximum end-point
                    if (!isMin) {
                        // The minimum end-point is moving to the left and
                        // passed a maximum end-point. Thus, the boxes start
                        // overlapping on the current axis. Therefore we test
                        // for box intersection
                        if (!box.equals(id1)) {
                            if (testIntersect2D(box, id1, otherAxis1, otherAxis2)
                                    && testIntersect1DSortedAABBs(id1, aabbInt, startEndPointsCurrentAxis, axis)) {

                                // Add an overlapping pair to the pair manager
                                pairManager.addPair(body, id1.body);
                            }
                        }

                        id1.max[axis] = indexEndPoint--;
                    } else {
                        id1.min[axis] = indexEndPoint--;
                    }

                    startEndPointsCurrentAxis[currentMinEndPointIndex + ((@Dimensionless int) (1))] = currentMinEndPoint;
                }

                // Update the current minimum endpoint that we are moving
                if (savedEndPointIndex != indexEndPoint) {
                    if (savedEndPoint.isMin) {
                        boxes[savedEndPoint.boxID].min[axis] = indexEndPoint;
                    } else {
                        boxes[savedEndPoint.boxID].max[axis] = indexEndPoint;
                    }

                    startEndPointsCurrentAxis[indexEndPoint] = savedEndPoint;
                }
            } else if (limit > currentMinEndPoint.value) {  // The minimum of the box has moved to the right

                currentMinEndPoint.value = limit;

                // The minimum end-point is moving right
                @Dimensionless
                EndPoint savedEndPoint = currentMinEndPoint;
                @Dimensionless
                int indexEndPoint = currentMinEndPointIndex;
                @Dimensionless
                int savedEndPointIndex = indexEndPoint;

                // For each end-point between the current position of the minimum
                // end-point and the new position of the minimum end-point
                while ((currentMinEndPoint = startEndPointsCurrentAxis[++currentMinEndPointIndex]).value < limit) {

                    @Dimensionless
                    BoxAABB id1 = boxes[currentMinEndPoint.boxID];
                    @Dimensionless
                    boolean isMin = currentMinEndPoint.isMin;

                    // If it's a maximum end-point
                    if (!isMin) {
                        // The minimum end-point is moving to the right and
                        // passed a maximum end-point. Thus, the boxes stop
                        // overlapping on the current axis.
                        if (!box.equals(id1)) {
                            if (testIntersect2D(box, id1, otherAxis1, otherAxis2)) {

                                // Remove the pair from the pair manager
                                pairManager.removePair(body.getBodyID(), id1.body.getBodyID());
                            }
                        }

                        id1.max[axis] = indexEndPoint++;
                    } else {
                        id1.min[axis] = indexEndPoint++;
                    }

                    startEndPointsCurrentAxis[currentMinEndPointIndex - ((@Dimensionless int) (1))] = currentMinEndPoint;
                }

                // Update the current minimum endpoint that we are moving
                if (savedEndPointIndex != indexEndPoint) {
                    if (savedEndPoint.isMin) {
                        boxes[savedEndPoint.boxID].min[axis] = indexEndPoint;
                    } else {
                        boxes[savedEndPoint.boxID].max[axis] = indexEndPoint;
                    }

                    startEndPointsCurrentAxis[indexEndPoint] = savedEndPoint;
                }
            }

            // ------- Update the maximum end-point ------------ //
            @Dimensionless
            EndPoint currentMaxEndPoint = startEndPointsCurrentAxis[box.max[axis]];

            @Dimensionless
            int currentMaxEndPointIndex = box.max[axis];
            assert (!currentMaxEndPoint.isMin);

            // Get the maximum value of the AABB on the current axis
            limit = aabbInt.max[axis];

            // If the new maximum value of the AABB is larger
            // than the current maximum end-point value. It means
            // that the AABB is moving to the right.
            if (limit > currentMaxEndPoint.value) {

                currentMaxEndPoint.value = limit;

                @Dimensionless
                EndPoint savedEndPoint = currentMaxEndPoint;
                @Dimensionless
                int indexEndPoint = currentMaxEndPointIndex;
                @Dimensionless
                int savedEndPointIndex = indexEndPoint;

                while ((currentMaxEndPoint = startEndPointsCurrentAxis[++currentMaxEndPointIndex]).value < limit) {

                    // Get the next end-point
                    @Dimensionless
                    BoxAABB id1 = boxes[currentMaxEndPoint.boxID];
                    @Dimensionless
                    boolean isMin = currentMaxEndPoint.isMin;

                    // If it's a maximum end-point
                    if (isMin) {
                        // The maximum end-point is moving to the right and
                        // passed a minimum end-point. Thus, the boxes start
                        // overlapping on the current axis. Therefore we test
                        // for box intersection
                        if (!box.equals(id1)) {
                            if (testIntersect2D(box, id1, otherAxis1, otherAxis2)
                                    && testIntersect1DSortedAABBs(id1, aabbInt, startEndPointsCurrentAxis, axis)) {

                                // Add an overlapping pair to the pair manager
                                pairManager.addPair(body, id1.body);
                            }
                        }

                        id1.min[axis] = indexEndPoint++;
                    } else {
                        id1.max[axis] = indexEndPoint++;
                    }

                    startEndPointsCurrentAxis[currentMaxEndPointIndex - ((@Dimensionless int) (1))] = currentMaxEndPoint;
                }

                // Update the current minimum endpoint that we are moving
                if (savedEndPointIndex != indexEndPoint) {
                    if (savedEndPoint.isMin) {
                        boxes[savedEndPoint.boxID].min[axis] = indexEndPoint;
                    } else {
                        boxes[savedEndPoint.boxID].max[axis] = indexEndPoint;
                    }

                    startEndPointsCurrentAxis[indexEndPoint] = savedEndPoint;
                }
            } else if (limit < currentMaxEndPoint.value) {  // If the AABB is moving to the left 

                currentMaxEndPoint.value = limit;

                @Dimensionless
                EndPoint savedEndPoint = currentMaxEndPoint;
                @Dimensionless
                int indexEndPoint = currentMaxEndPointIndex;
                @Dimensionless
                int savedEndPointIndex = indexEndPoint;

                // For each end-point between the current position of the maximum
                // end-point and the new position of the maximum end-point
                while ((currentMaxEndPoint = startEndPointsCurrentAxis[--currentMaxEndPointIndex]).value > limit) {
                    @Dimensionless
                    BoxAABB id1 = boxes[currentMaxEndPoint.boxID];
                    @Dimensionless
                    boolean isMin = currentMaxEndPoint.isMin;

                    // If it's a minimum end-point
                    if (isMin) {
                        // The maximum end-point is moving to the right and
                        // passed a minimum end-point. Thus, the boxes stop
                        // overlapping on the current axis.
                        if (!box.equals(id1)) {
                            if (testIntersect2D(box, id1, otherAxis1, otherAxis2)) {

                                // Remove the pair from the pair manager
                                pairManager.removePair(body.getBodyID(), id1.body.getBodyID());
                            }
                        }

                        id1.min[axis] = indexEndPoint--;
                    } else {
                        id1.max[axis] = indexEndPoint--;
                    }

                    startEndPointsCurrentAxis[currentMaxEndPointIndex + ((@Dimensionless int) (1))] = currentMaxEndPoint;
                }

                // Update the current minimum endpoint that we are moving
                if (savedEndPointIndex != indexEndPoint) {
                    if (savedEndPoint.isMin) {
                        boxes[savedEndPoint.boxID].min[axis] = indexEndPoint;
                    } else {
                        boxes[savedEndPoint.boxID].max[axis] = indexEndPoint;
                    }

                    startEndPointsCurrentAxis[indexEndPoint] = savedEndPoint;
                }
            }
        }
    }

    // Notify the broad-phase about a new object in the world
    // This method adds the AABB of the object ion to broad-phase
    @Override
    public void addObject(@Dimensionless SweepAndPruneAlgorithm this, @Dimensionless CollisionBody body, @Dimensionless AABB aabb) {

        @Dimensionless
        int boxIndex;

        assert (Utils.encodeFloatIntoInteger(aabb.getMin().getX()) > Utils.encodeFloatIntoInteger(- ((@Dimensionless float) (Float.MAX_VALUE))));
        assert (Utils.encodeFloatIntoInteger(aabb.getMin().getY()) > Utils.encodeFloatIntoInteger(- ((@Dimensionless float) (Float.MAX_VALUE))));
        assert (Utils.encodeFloatIntoInteger(aabb.getMin().getZ()) > Utils.encodeFloatIntoInteger(- ((@Dimensionless float) (Float.MAX_VALUE))));
        assert (Utils.encodeFloatIntoInteger(aabb.getMax().getX()) < Utils.encodeFloatIntoInteger(((@Dimensionless float) (Float.MAX_VALUE))));
        assert (Utils.encodeFloatIntoInteger(aabb.getMax().getY()) < Utils.encodeFloatIntoInteger(((@Dimensionless float) (Float.MAX_VALUE))));
        assert (Utils.encodeFloatIntoInteger(aabb.getMax().getZ()) < Utils.encodeFloatIntoInteger(((@Dimensionless float) (Float.MAX_VALUE))));

        // If the index of the first free box is valid (means that
        // there is a bucket in the middle of the array that doesn't
        // contain a box anymore because it has been removed)
        if (!freeBoxIndices.isEmpty()) {
            @Dimensionless
            int lastIndex = freeBoxIndices.size() - ((@Dimensionless int) (1));
            boxIndex = freeBoxIndices.get(lastIndex);
            freeBoxIndices.remove(lastIndex);
        } else {
            // If the array boxes and end-points arrays are full
            if (numBoxes == numMaxBoxes) {
                // Resize the arrays to make them larger
                resizeArrays();
            }

            boxIndex = numBoxes;
        }

        // Move the maximum limit end-point two elements further
        // at the end-points array in all three axis
        @Dimensionless
        int indexLimitEndPoint = ((@Dimensionless int) (2)) * numBoxes + NUM_SENTINELS - ((@Dimensionless int) (1));
        for (@Dimensionless int axis = ((@Dimensionless int) (0)); axis < ((@Dimensionless int) (3)); axis++) {
            @Dimensionless
            EndPoint maxLimitEndPoint = endPoints[axis][indexLimitEndPoint];
            assert (endPoints[axis][((@Dimensionless int) (0))].boxID == INVALID_INDEX && endPoints[axis][((@Dimensionless int) (0))].isMin);
            assert (maxLimitEndPoint.boxID == INVALID_INDEX && !maxLimitEndPoint.isMin);
            if (endPoints[axis][indexLimitEndPoint + ((@Dimensionless int) (2))] == null) {
                endPoints[axis][indexLimitEndPoint + ((@Dimensionless int) (2))] = new @Dimensionless EndPoint();
            }
            @Dimensionless
            EndPoint newMaxLimitEndPoint = endPoints[axis][indexLimitEndPoint + ((@Dimensionless int) (2))];
            newMaxLimitEndPoint.setValues(maxLimitEndPoint.boxID, maxLimitEndPoint.isMin,
                    maxLimitEndPoint.value);
        }

        // Create a new box
        if (boxes[boxIndex] == null) {
            boxes[boxIndex] = new @Dimensionless BoxAABB();
        }
        @Dimensionless
        BoxAABB box = boxes[boxIndex];
        box.body = body;
        @Dimensionless
        long maxEndPointValue = Utils.encodeFloatIntoInteger(((@Dimensionless float) (Float.MAX_VALUE))) - ((@Dimensionless int) (1));
        @Dimensionless
        long minEndPointValue = Utils.encodeFloatIntoInteger(((@Dimensionless float) (Float.MAX_VALUE))) - ((@Dimensionless int) (2));
        for (@Dimensionless int axis = ((@Dimensionless int) (0)); axis < ((@Dimensionless int) (3)); axis++) {
            box.min[axis] = indexLimitEndPoint;
            box.max[axis] = indexLimitEndPoint + ((@Dimensionless int) (1));
            @Dimensionless
            EndPoint minimumEndPoint = endPoints[axis][box.min[axis]];
            minimumEndPoint.setValues(boxIndex, true, minEndPointValue);
            if (endPoints[axis][box.max[axis]] == null) {
                endPoints[axis][box.max[axis]] = new @Dimensionless EndPoint();
            }
            @Dimensionless
            EndPoint maximumEndPoint = endPoints[axis][box.max[axis]];
            maximumEndPoint.setValues(boxIndex, false, maxEndPointValue);
        }

        // Add the body pointer to box index mapping
        mapBodyToBoxIndex.put(body, boxIndex);

        numBoxes++;

        // Call the update method to put the end-points of the new AABB at the
        // correct position in the array. This will also create the overlapping
        // pairs in the pair manager if the new AABB is overlapping with others
        // AABBs
        updateObject(body, aabb);
    }

    // Notify the broad-phase about an object that has been removed from the world
    @Override
    public void removeObject(@Dimensionless SweepAndPruneAlgorithm this, @Dimensionless CollisionBody body) {

        assert (numBoxes > ((@Dimensionless int) (0)));

        // Call the update method with an AABB that is very far away
        // in order to remove all overlapping pairs from the pair manager
        @Dimensionless
        long maxEndPointValue = Utils.encodeFloatIntoInteger(((@Dimensionless float) (Float.MAX_VALUE))) - ((@Dimensionless int) (1));
        @Dimensionless
        long minEndPointValue = Utils.encodeFloatIntoInteger(((@Dimensionless float) (Float.MAX_VALUE))) - ((@Dimensionless int) (2));
        @Dimensionless
        AABBInt aabbInt = new @Dimensionless AABBInt(minEndPointValue, maxEndPointValue);
        updateObjectIntegerAABB(body, aabbInt);

        // Get the corresponding box
        @Dimensionless
        int boxIndex = mapBodyToBoxIndex.get(body);

        // Remove the end-points of the box by moving the maximum end-points two elements back in
        // the end-points array
        @Dimensionless
        int indexLimitEndPoint = ((@Dimensionless int) (2)) * numBoxes + NUM_SENTINELS - ((@Dimensionless int) (1));
        for (@Dimensionless int axis = ((@Dimensionless int) (0)); axis < ((@Dimensionless int) (3)); axis++) {
            @Dimensionless
            EndPoint maxLimitEndPoint = endPoints[axis][indexLimitEndPoint];
            assert (endPoints[axis][((@Dimensionless int) (0))].boxID == INVALID_INDEX && endPoints[axis][((@Dimensionless int) (0))].isMin);
            assert (maxLimitEndPoint.boxID == INVALID_INDEX && !maxLimitEndPoint.isMin);
            @Dimensionless
            EndPoint newMaxLimitEndPoint = endPoints[axis][indexLimitEndPoint - ((@Dimensionless int) (2))];
            assert (endPoints[axis][indexLimitEndPoint - ((@Dimensionless int) (1))].boxID == boxIndex);
            assert (!endPoints[axis][indexLimitEndPoint - ((@Dimensionless int) (1))].isMin);
            assert (newMaxLimitEndPoint.boxID == boxIndex);
            assert (newMaxLimitEndPoint.isMin);
            newMaxLimitEndPoint.setValues(maxLimitEndPoint.boxID, maxLimitEndPoint.isMin, maxLimitEndPoint.value);
        }

        // Add the box index into the list of free indices
        freeBoxIndices.add(boxIndex);

        mapBodyToBoxIndex.remove(body);
        numBoxes--;

        // Check if we need to shrink the allocated memory
        @Dimensionless
        int nextPowerOf2 = PairManager.ComputeNextPowerOfTwo((numBoxes - ((@Dimensionless int) (1))) / ((@Dimensionless int) (100)));
        if (nextPowerOf2 * ((@Dimensionless int) (100)) < numMaxBoxes) {
            shrinkArrays();
        }
    }

    // Notify the broad-phase that the AABB of an object has changed
    @Override
    public void updateObject(@Dimensionless SweepAndPruneAlgorithm this, @Dimensionless CollisionBody body, @Dimensionless AABB aabb) {

        // Compute the corresponding AABB with integer coordinates
        @Dimensionless
        AABBInt aabbInt = new @Dimensionless AABBInt(aabb);

        // Call the update object method that uses an AABB with integer coordinates
        updateObjectIntegerAABB(body, aabbInt);
    }

}
