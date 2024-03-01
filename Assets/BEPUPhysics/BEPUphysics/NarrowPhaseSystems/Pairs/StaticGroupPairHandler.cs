﻿using System;
using BEPUphysics.BroadPhaseEntries;

namespace BEPUphysics.NarrowPhaseSystems.Pairs
{
    ///<summary>
    /// Handles a compound and group collision pair.
    ///</summary>
    public abstract class StaticGroupPairHandler : GroupPairHandler
    {
        protected StaticGroup staticGroup;

        public override Collidable CollidableA
        {
            get { return staticGroup; }
        }
        public override Entities.Entity EntityA
        {
            get { return null; }
        }



        ///<summary>
        /// Initializes the pair handler.
        ///</summary>
        ///<param name="entryA">First entry in the pair.</param>
        ///<param name="entryB">Second entry in the pair.</param>
        public override void Initialize(BroadPhaseEntry entryA, BroadPhaseEntry entryB)
        {
            //Other member of the pair is initialized by the child.
            staticGroup = entryA as StaticGroup;
            if (staticGroup == null)
            {
                staticGroup = entryB as StaticGroup;
                if (staticGroup == null)
                {
                    throw new ArgumentException("Inappropriate types used to initialize pair.");
                }
            }

            base.Initialize(entryA, entryB);
        }

        ///<summary>
        /// Cleans up the pair handler.
        ///</summary>
        public override void CleanUp()
        {
            base.CleanUp();

            staticGroup = null;
            //Child type needs to null out other reference.
        }



    }
}
