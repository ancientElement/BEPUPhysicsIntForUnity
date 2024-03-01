﻿using FixMath.NET;

namespace BEPUphysics.UpdateableSystems
{
    ///<summary>
    /// Defines an object which is updated by the space at the end of a time step.
    ///</summary>
    public interface IEndOfTimeStepUpdateable : ISpaceUpdateable
    {

        ///<summary>
        /// Updates the object at the end of a time step.
        ///</summary>
        ///<param name="dt">Time step duration.</param>
        void Update(Fix64 dt);

    }
}
