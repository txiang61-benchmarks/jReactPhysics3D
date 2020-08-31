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
import units.qual.ns;
import units.qual.Dimensionless;

/**
 * This class will take care of the time in the physics engine. It uses functions that depend on the current platform to
 * get the current time.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Timer {

    // True if the timer is running
    private @Dimensionless boolean isRunning;

    // Used to fix the time step and avoid strange time effects
    private @Dimensionless double accumulator;

    // Timestep dt of the physics engine (timestep > 0.0f)
    private @Dimensionless double timeStep;

    // Time difference between the two last timer update() calls
    private @Dimensionless float deltaTime;

    // Last time the timer has been updated
    private @Dimensionless float lastUpdateTime;

    // Constructor
    public Timer(double timeStep) {
        assert (timeStep > ((@Dimensionless float) (0.0f)));
        isRunning = false;
        this.timeStep = timeStep;
    }

    // Compute the interpolation factor
    public float computeInterpolationFactor(@Dimensionless Timer this) {
        return (@Dimensionless float) (accumulator / timeStep);
    }

    // Return if the timer is running
    public boolean getIsRunning(@Dimensionless Timer this) {
        return isRunning;
    }

    // Return the timestep of the physics engine
    public double getTimeStep(@Dimensionless Timer this) {
        return timeStep;
    }

    // Set the timestep of the physics engine
    public void setTimeStep(@Dimensionless Timer this, @Dimensionless double timeStep) {
        assert (timeStep > ((@Dimensionless float) (0.0f)));
        this.timeStep = timeStep;
    }

    // Return the current time
    public float getPhysicsTime(@Dimensionless Timer this) {
        return lastUpdateTime;
    }

    // True if it's possible to take a new step
    public boolean isPossibleToTakeStep(@Dimensionless Timer this) {
        return (accumulator >= timeStep);
    }

    // Take a new step => update the timer by adding the timeStep value to the current time
    public void nextStep(@Dimensionless Timer this) {
        assert (isRunning);

        // Update the accumulator value
        accumulator -= timeStep;
    }

    // Start the timer
    public void start(@Dimensionless Timer this) {
        if (!isRunning) {

            isRunning = true;
            accumulator = ((@Dimensionless double) (0.0));
            // Get the current system time
            lastUpdateTime = GetCurrentSystemTime();
        }
    }

    // Stop the timer
    public void stop(@Dimensionless Timer this) {
        isRunning = false;
    }

    // Compute the time since the last update() call and add it to the accumulator
    public void update(@Dimensionless Timer this) {

        // Get the current system time
        @Dimensionless
        float currentTime = GetCurrentSystemTime();

        // Compute the delta display time between two display frames
        deltaTime = currentTime - lastUpdateTime;

        // Update the current display time
        lastUpdateTime = currentTime;

        // Update the accumulator value
        accumulator += deltaTime;
    }

    // Return the current time of the system in seconds
    public static float GetCurrentSystemTime() {
        return System.nanoTime() / ((@ns float) (1000000000.0f));
    }

}
