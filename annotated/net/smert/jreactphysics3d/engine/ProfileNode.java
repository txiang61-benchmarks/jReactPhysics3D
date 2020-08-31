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
package net.smert.jreactphysics3d.engine;
import units.qual.Dimensionless;

/**
 * It represents a profile sample in the profiler tree.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
@Dimensionless
public class ProfileNode {

    // Starting time of the sampling of corresponding block of code
    private @Dimensionless float startingTime;

    // Total time spent in the block of code
    private @Dimensionless float totalTime;

    // Total number of calls of this node
    private @Dimensionless int numTotalCalls;

    // Recursion counter
    private @Dimensionless int recursionCounter;

    // Pointer to a child node
    private @Dimensionless ProfileNode childNode;

    // Pointer to the parent node
    private final @Dimensionless ProfileNode parentNode;

    // Pointer to a sibling node
    private @Dimensionless ProfileNode siblingNode;

    // Name of the node
    private final @Dimensionless String name;

    // Constructor
    public ProfileNode(@Dimensionless String name, @Dimensionless ProfileNode parentNode) {
        assert (name != null);
        startingTime = ((@Dimensionless long) (0l));
        totalTime = ((@Dimensionless long) (0l));
        numTotalCalls = ((@Dimensionless int) (0));
        recursionCounter = ((@Dimensionless int) (0));
        childNode = null;
        this.parentNode = parentNode;
        siblingNode = null;
        this.name = name;
        reset();
    }

    // Destroy the node
    public void destroy(@Dimensionless ProfileNode this) {
        //delete mChildNode;
        childNode = null;
        //delete mSiblingNode;
        siblingNode = null;
    }

    // Called when we enter the block of code corresponding to this profile node
    public void enterBlockOfCode(@Dimensionless ProfileNode this) {
        numTotalCalls++;

        // If the current code is not called recursively
        if (recursionCounter == ((@Dimensionless int) (0))) {

            // Get the current system time to initialize the starting time of
            // the profiling of the current block of code
            startingTime = Timer.GetCurrentSystemTime() * ((@Dimensionless float) (1000.0f));
        }

        recursionCounter++;
    }

    // Called when we exit the block of code corresponding to this profile node
    public @Dimensionless boolean exitBlockOfCode(@Dimensionless ProfileNode this) {
        recursionCounter--;

        if (recursionCounter == ((@Dimensionless int) (0)) && numTotalCalls != ((@Dimensionless int) (0))) {

            // Get the current system time
            @Dimensionless
            float currentTime = Timer.GetCurrentSystemTime() * ((@Dimensionless float) (1000.0f));

            // Increase the total elasped time in the current block of code
            totalTime += currentTime - startingTime;
        }

        // Return true if the current code is not recursing
        return (recursionCounter == ((@Dimensionless int) (0)));
    }

    // Return a pointer to a sub node with a given name
    public @Dimensionless ProfileNode findSubNode(@Dimensionless ProfileNode this, @Dimensionless String name) {

        // Try to find the node among the child nodes
        @Dimensionless
        ProfileNode child = childNode;
        while (child != null) {
            if (child.name.equals(name)) {
                return child;
            }
            child = child.siblingNode;
        }

        // The nose has not been found. Therefore, we create it
        // and add it to the profiler tree
        @Dimensionless
        ProfileNode newNode = new @Dimensionless ProfileNode(name, this);
        newNode.siblingNode = childNode;
        childNode = newNode;

        return newNode;
    }

    // Return the total time spent in the block of code
    public @Dimensionless float getTotalTime(@Dimensionless ProfileNode this) {
        return totalTime;
    }

    // Return the total number of call of the corresponding block of code
    public @Dimensionless int getNumTotalCalls(@Dimensionless ProfileNode this) {
        return numTotalCalls;
    }

    // Return a pointer to a child node
    public @Dimensionless ProfileNode getChildNode(@Dimensionless ProfileNode this) {
        return childNode;
    }

    // Return a pointer to the parent node
    public @Dimensionless ProfileNode getParentNode(@Dimensionless ProfileNode this) {
        return parentNode;
    }

    // Return a pointer to a sibling node
    public @Dimensionless ProfileNode getSiblingNode(@Dimensionless ProfileNode this) {
        return siblingNode;
    }

    // Return the name of the node
    public @Dimensionless String getName(@Dimensionless ProfileNode this) {
        return name;
    }

    // Reset the profiling of the node
    public final void reset(@Dimensionless ProfileNode this) {
        totalTime = ((@Dimensionless int) (0));
        numTotalCalls = ((@Dimensionless int) (0));

        // Reset the child node
        if (childNode != null) {
            childNode.reset();
        }

        // Reset the sibling node
        if (siblingNode != null) {
            siblingNode.reset();
        }
    }

}
