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
package net.smert.jreactphysics3d.body;
import units.qual.Dimensionless;

/**
 * This class is an abstract class to represent a body of the physics engine.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public abstract class Body {

    // True if the body is active
    protected @Dimensionless boolean isActive;

    // True if the body is allowed to go to sleep for better efficiency
    protected @Dimensionless boolean isAllowedToSleep;

    // True if the body has already been added in an island (for sleeping technique)
    protected @Dimensionless boolean isAlreadyInIsland;

    // True if the body is sleeping (for sleeping technique)
    protected boolean isSleeping;

    // Elapsed time since the body velocity was bellow the sleep velocity
    protected @Dimensionless float sleepTime;

    // Unique ID of the body
    protected @Dimensionless int bodyID;

    // Constructor
    public Body(int bodyID) {
        assert (bodyID >= ((@Dimensionless int) (0)));
        isActive = true;
        isAllowedToSleep = true;
        isAlreadyInIsland = false;
        isSleeping = false;
        sleepTime = ((@Dimensionless int) (0));
        this.bodyID = bodyID;
    }

    // Return true if the body is active
    public @Dimensionless boolean isActive(@Dimensionless Body this) {
        return isActive;
    }

    // Return whether or not the body is allowed to sleep
    public @Dimensionless boolean isAllowedToSleep(@Dimensionless Body this) {
        return isAllowedToSleep;
    }

    // Set whether or not the body is allowed to go to sleep
    public void setIsAllowedToSleep(@Dimensionless Body this, @Dimensionless boolean isAllowedToSleep) {
        this.isAllowedToSleep = isAllowedToSleep;

        if (!this.isAllowedToSleep) {
            setIsSleeping(false);
        }
    }

    public @Dimensionless boolean isAlreadyInIsland(@Dimensionless Body this) {
        return isAlreadyInIsland;
    }

    public void setIsAlreadyInIsland(@Dimensionless Body this, @Dimensionless boolean isAlreadyInIsland) {
        this.isAlreadyInIsland = isAlreadyInIsland;
    }

    // Return whether or not the body is sleeping
    public @Dimensionless boolean isSleeping(@Dimensionless Body this) {
        return isSleeping;
    }

    // Set the variable to know whether or not the body is sleeping
    public void setIsSleeping(@Dimensionless Body this, boolean isSleeping) {

        if (isSleeping) {
            sleepTime = ((@Dimensionless float) (0.0f));
        } else {
            if (this.isSleeping) {
                sleepTime = ((@Dimensionless float) (0.0f));
            }
        }

        this.isSleeping = isSleeping;
    }

    public @Dimensionless float getSleepTime(@Dimensionless Body this) {
        return sleepTime;
    }

    public void setSleepTime(@Dimensionless Body this, @Dimensionless float sleepTime) {
        this.sleepTime = sleepTime;
    }

    // Return the id of the body
    public @Dimensionless int getBodyID(@Dimensionless Body this) {
        return bodyID;
    }

}
