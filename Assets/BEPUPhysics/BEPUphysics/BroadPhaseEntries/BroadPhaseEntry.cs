﻿using System;
using BEPUphysics.BroadPhaseSystems;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUutilities;
using FixMath.NET;

namespace BEPUphysics.BroadPhaseEntries
{
    /// <summary>
    /// Superclass of all objects which live inside the broad phase.
    /// The BroadPhase will generate pairs between BroadPhaseEntries.
    /// </summary>
    public abstract class BroadPhaseEntry : IBoundingBoxOwner, ICollisionRulesOwner
    {
        internal int hashCode;
        protected BroadPhaseEntry()
        {
            CollisionRules = new CollisionRules();
            collisionRulesUpdatedDelegate = CollisionRulesUpdated;

            hashCode = (int)(base.GetHashCode() * 0xd8163841);
        }

        /// <summary>
        /// Gets the broad phase to which this broad phase entry belongs.
        /// </summary>
        public BroadPhase BroadPhase
        {
            get;
            internal set;
        }

        /// <summary>
        /// Gets the object's hash code.
        /// </summary>
        /// <returns>Hash code for the object.</returns>
        public override int GetHashCode()
        {
            return hashCode;
        }

        private Action collisionRulesUpdatedDelegate;
        protected abstract void CollisionRulesUpdated();

        protected internal BoundingBox boundingBox;
        /// <summary>
        /// Gets or sets the bounding box of the entry.
        /// </summary>
        public BoundingBox BoundingBox
        {
            get { return boundingBox; }
            set
            {
                boundingBox = value;
            }
        }

        /// <summary>
        /// Gets whether this collidable is associated with an active entity. True if it is, false if it's not.
        /// </summary>
        public abstract bool IsActive { get; }

        internal CollisionRules collisionRules;
        /// <summary>
        /// Gets the entry's collision rules.
        /// </summary>
        public CollisionRules CollisionRules
        {
            get { return collisionRules; }
            set
            {
                if (collisionRules != value)
                {
                    if (collisionRules != null)
                        collisionRules.CollisionRulesChanged -= collisionRulesUpdatedDelegate;
                    collisionRules = value;
                    if (collisionRules != null)
                        collisionRules.CollisionRulesChanged += collisionRulesUpdatedDelegate;
                    CollisionRulesUpdated();
                }
            }
        }

        /// <summary>
        /// Tests a ray against the entry.
        /// </summary>
        /// <param name="ray">Ray to test.</param>
        /// <param name="maximumLength">Maximum length, in units of the ray's direction's length, to test.</param>
        /// <param name="rayHit">Hit location of the ray on the entry, if any.</param>
        /// <returns>Whether or not the ray hit the entry.</returns>
        public abstract bool RayCast(Ray ray, Fix64 maximumLength, out RayHit rayHit);

        /// <summary>
        /// Tests a ray against the entry.
        /// </summary>
        /// <param name="ray">Ray to test.</param>
        /// <param name="maximumLength">Maximum length, in units of the ray's direction's length, to test.</param>
        /// <param name="filter">Test to apply to the entry. If it returns true, the entry is processed, otherwise the entry is ignored. If a collidable hierarchy is present
        /// in the entry, this filter will be passed into inner ray casts.</param>
        /// <param name="rayHit">Hit location of the ray on the entry, if any.</param>
        /// <returns>Whether or not the ray hit the entry.</returns>
        public virtual bool RayCast(Ray ray, Fix64 maximumLength, Func<BroadPhaseEntry, bool> filter, out RayHit rayHit)
        {
            if (filter(this))
                return RayCast(ray, maximumLength, out rayHit);
            rayHit = new RayHit();
            return false;
        }



        /// <summary>
        /// Sweeps a convex shape against the entry.
        /// </summary>
        /// <param name="castShape">Swept shape.</param>
        /// <param name="startingTransform">Beginning location and orientation of the cast shape.</param>
        /// <param name="sweep">Sweep motion to apply to the cast shape.</param>
        /// <param name="hit">Hit data of the cast on the entry, if any.</param>
        /// <returns>Whether or not the cast hit the entry.</returns>
        public abstract bool ConvexCast(ConvexShape castShape, ref RigidTransform startingTransform, ref Vector3 sweep, out RayHit hit);

        /// <summary>
        /// Sweeps a convex shape against the entry.
        /// </summary>
        /// <param name="castShape">Swept shape.</param>
        /// <param name="startingTransform">Beginning location and orientation of the cast shape.</param>
        /// <param name="sweep">Sweep motion to apply to the cast shape.</param>
        /// <param name="filter">Test to apply to the entry. If it returns true, the entry is processed, otherwise the entry is ignored. If a collidable hierarchy is present
        /// in the entry, this filter will be passed into inner ray casts.</param>
        /// <param name="hit">Hit data of the cast on the entry, if any.</param>
        /// <returns>Whether or not the cast hit the entry.</returns>
        public virtual bool ConvexCast(ConvexShape castShape, ref RigidTransform startingTransform, ref Vector3 sweep, Func<BroadPhaseEntry, bool> filter, out RayHit hit)
        {
            if (filter(this))
                return ConvexCast(castShape, ref startingTransform, ref sweep, out hit);
            hit = new RayHit();
            return false;
        }

        /// <summary>
        /// Updates the bounding box to the current state of the entry.
        /// </summary>
        public abstract void UpdateBoundingBox();

        
        /// <summary>
        /// Gets or sets the user data associated with this entry.
        /// </summary>
        public object Tag { get; set; }


    }

}
