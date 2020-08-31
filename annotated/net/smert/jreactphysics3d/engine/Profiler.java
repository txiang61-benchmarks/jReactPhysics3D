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
import net.smert.jreactphysics3d.configuration.Defaults;

/**
 * This is the main class of the profiler. This profiler is based on "Real-Time Hierarchical Profiling" article from
 * "Game Programming Gems 3" by Greg Hjelstrom and Byon Garrabrant.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
@Dimensionless
public class Profiler {

    // Starting profiling time
    private static @Dimensionless float profilingStartTime;

    // Frame counter
    private static @Dimensionless int frameCounter;

    // Current node in the current execution
    private static @Dimensionless ProfileNode currentNode;

    // Root node of the profiler tree
    private static final @Dimensionless ProfileNode rootNode;

    // Destroy the profiler (release the memory)
    public static void Destroy() {
        rootNode.destroy();
    }

    // Return the elasped time since the start/reset of the profiling
    public static @Dimensionless float GetElapsedTimeSinceStart() {
        @Dimensionless
        float currentTime = Timer.GetCurrentSystemTime() * ((@Dimensionless long) (1000l));
        return currentTime - profilingStartTime;
    }

    // Return the number of frames
    public static @Dimensionless int getNumFrames() {
        return frameCounter;
    }

    // Return an iterator over the profiler tree starting at the root
    public static @Dimensionless ProfileNodeIterator GetIterator() {
        return new @Dimensionless ProfileNodeIterator(rootNode);
    }

    // Increment the frame counter
    public static void IncrementFrameCounter() {
        frameCounter++;
    }

    // Recursively print the report of a given node of the profiler tree
    private static void PrintRecursiveNodeReport(@Dimensionless ProfileNodeIterator iterator, @Dimensionless int spacing, @Dimensionless int outputStream) {
        iterator.first();

        // If we are at the end of a branch in the profiler tree
        if (iterator.isEnd()) {
            return;
        }

        @Dimensionless
        float parentTime = iterator.isRoot() ? GetElapsedTimeSinceStart() : iterator.getCurrentParentTotalTime();
        @Dimensionless
        long accumulatedTime = ((@Dimensionless long) (0l));
        @Dimensionless
        int nbFrames = getNumFrames();
        for (@Dimensionless int i = ((@Dimensionless int) (0)); i < spacing; i++) {
            System.out.print(" ");
        }
        System.out.println("---------------");
        for (@Dimensionless int i = ((@Dimensionless int) (0)); i < spacing; i++) {
            System.out.print(" ");
        }
        System.out.print("| Profiling : ");
        System.out.print(iterator.getCurrentParentName());
        System.out.print(" (total running time : ");
        System.out.print(parentTime);
        System.out.println(" ms) ---");
        @Dimensionless
        long totalTime = ((@Dimensionless long) (0l));

        // Recurse over the children of the current node
        @Dimensionless
        int nbChildren = ((@Dimensionless int) (0));
        for (@Dimensionless int i = ((@Dimensionless int) (0)); !iterator.isEnd(); i++, iterator.next()) {
            nbChildren++;
            @Dimensionless
            float currentTotalTime = iterator.getCurrentTotalTime();
            accumulatedTime += currentTotalTime;
            @Dimensionless
            long fraction = parentTime > Defaults.MACHINE_EPSILON ? (@Dimensionless long) (currentTotalTime / parentTime) * ((@Dimensionless long) (100l)) : ((@Dimensionless long) (0l));
            for (@Dimensionless int j = ((@Dimensionless int) (0)); j < spacing; j++) {
                System.out.print(" ");
            }
            System.out.print("|   ");
            System.out.print(i);
            System.out.print(" -- ");
            System.out.print(iterator.getCurrentName());
            System.out.print(" : ");
            System.out.print(fraction);
            System.out.print(" % | ");
            System.out.print(currentTotalTime / (@Dimensionless long) nbFrames);
            System.out.print(" ms/frame (");
            System.out.print(iterator.getCurrentNumTotalCalls());
            System.out.println(" calls)");
            totalTime += currentTotalTime;
        }

        if (parentTime < accumulatedTime) {
            System.out.println("Something is wrong !");
        }
        for (@Dimensionless int i = ((@Dimensionless int) (0)); i < spacing; i++) {
            System.out.print(" ");
        }
        @Dimensionless
        long percentage = parentTime > Defaults.MACHINE_EPSILON ? (@Dimensionless long) ((parentTime - accumulatedTime) / parentTime) * ((@Dimensionless long) (100l)) : ((@Dimensionless long) (0l));
        @Dimensionless
        float difference = parentTime - accumulatedTime;
        System.out.print("| Unaccounted : ");
        System.out.print(difference);
        System.out.print(" ms (");
        System.out.print(percentage);
        System.out.println(" %)");

        for (@Dimensionless int i = ((@Dimensionless int) (0)); i < nbChildren; i++) {
            iterator.enterChild(i);
            PrintRecursiveNodeReport(iterator, spacing + ((@Dimensionless int) (3)), outputStream);
            iterator.enterParent();
        }
    }

    // Print the report of the profiler in a given output stream
    public static void PrintReport(@Dimensionless int outputStream) {
        @Dimensionless
        ProfileNodeIterator iterator = GetIterator();

        // Recursively print the report of each node of the profiler tree
        PrintRecursiveNodeReport(iterator, ((@Dimensionless int) (0)), outputStream);

        // Destroy the iterator
    }

    // Reset the timing data of the profiler (but not the profiler tree structure)
    public static void Reset() {
        rootNode.reset();
        rootNode.enterBlockOfCode();
        frameCounter = ((@Dimensionless int) (0));
        profilingStartTime = Timer.GetCurrentSystemTime() * ((@Dimensionless float) (1000.0f));
    }

    // Method called when we want to start profiling a block of code.
    public static void StartProfilingBlock(@Dimensionless String name) {

        // Look for the node in the tree that corresponds to the block of
        // code to profile
        if (!name.equals(currentNode.getName())) {
            currentNode = currentNode.findSubNode(name);
        }

        // Start profile the node
        currentNode.enterBlockOfCode();
    }

    // Method called at the end of the scope where the
    // startProfilingBlock() method has been called.
    public static void StopProfilingBlock() {

        // Go to the parent node unless if the current block
        // of code is recursing
        if (currentNode.exitBlockOfCode()) {
            currentNode = currentNode.getParentNode();
        }
    }

    // Initialization of static variables
    static {
        rootNode = new @Dimensionless ProfileNode("Root", null);
        currentNode = rootNode;
        profilingStartTime = Timer.GetCurrentSystemTime() * ((@Dimensionless float) (1000.0f));
        frameCounter = ((@Dimensionless int) (0));
    }

}
