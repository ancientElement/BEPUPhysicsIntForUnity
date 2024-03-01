﻿using System;
using BEPUphysics.BroadPhaseEntries.Events;
using BEPUphysics.CollisionShapes;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUutilities;
using BEPUphysics.OtherSpaceStages;
using System.Collections.Generic;
using RigidTransform = BEPUutilities.RigidTransform;
using FixMath.NET;

namespace BEPUphysics.BroadPhaseEntries
{
    ///<summary>
    /// Collection of unmoving collidable objects.
    ///</summary>
    ///<remarks>
    /// Batching multiple static objects together into a StaticGroup as opposed to adding them separately to the Space avoids BroadPhase pollution, improving performance.
    /// </remarks>
    public class StaticGroup : StaticCollidable
    {


        ///<summary>
        /// Constructs a new static mesh.
        ///</summary>
        ///<param name="collidables">List of collidables in the static group.</param>
        public StaticGroup(IList<Collidable> collidables)
        {
            base.Shape = new StaticGroupShape(collidables, this);
            Events = new ContactEventManager<StaticGroup>();

        }

        ///<summary>
        /// Gets the shape used by the mesh.  Unlike most collidable-shape pairs, StaticGroupShapes cannot be shared between multiple StaticGroups.
        ///</summary>
        public new StaticGroupShape Shape
        {
            get
            {
                return (StaticGroupShape)shape;
            }
        }


        protected internal ContactEventManager<StaticGroup> events;

        ///<summary>
        /// Gets the event manager used by the mesh.
        ///</summary>
        public ContactEventManager<StaticGroup> Events
        {
            get
            {
                return events;
            }
            set
            {
                if (value.Owner != null && //Can't use a manager which is owned by a different entity.
                    value != events) //Stay quiet if for some reason the same event manager is being set.
                    throw new ArgumentException("Event manager is already owned by a mesh; event managers cannot be shared.");
                if (events != null)
                    events.Owner = null;
                events = value;
                if (events != null)
                    events.Owner = this;
            }
        }
        protected internal override IContactEventTriggerer EventTriggerer
        {
            get { return events; }
        }
        protected override IDeferredEventCreator EventCreator
        {
            get { return events; }
        }


        /// <summary>
        /// Updates the bounding box to the current state of the entry.
        /// </summary>
        public override void UpdateBoundingBox()
        {
            boundingBox = Shape.CollidableTree.BoundingBox;
        }

        /// <summary>
        /// Tests a ray against the entry.
        /// </summary>
        /// <param name="ray">Ray to test.</param>
        /// <param name="maximumLength">Maximum length, in units of the ray's direction's length, to test.</param>
        /// <param name="rayHit">Hit location of the ray on the entry, if any.</param>
        /// <returns>Whether or not the ray hit the entry.</returns>
        public override bool RayCast(Ray ray, Fix64 maximumLength, out RayHit rayHit)
        {
            RayCastResult result;
            bool toReturn = Shape.RayCast(ray, maximumLength, out result);
            rayHit = result.HitData;
            return toReturn;
        }

        /// <summary>
        /// Tests a ray against the entry.
        /// </summary>
        /// <param name="ray">Ray to test.</param>
        /// <param name="maximumLength">Maximum length, in units of the ray's direction's length, to test.</param>
        /// <param name="filter">Test to apply to the entry. If it returns true, the entry is processed, otherwise the entry is ignored. If a collidable hierarchy is present
        /// in the entry, this filter will be passed into inner ray casts.</param>
        /// <param name="rayHit">Hit location of the ray on the entry, if any.</param>
        /// <returns>Whether or not the ray hit the entry.</returns>
        public override bool RayCast(Ray ray, Fix64 maximumLength, Func<BroadPhaseEntry, bool> filter, out RayHit rayHit)
        {
            RayCastResult result;
            bool toReturn = Shape.RayCast(ray, maximumLength, filter, out result);
            rayHit = result.HitData;
            return toReturn;
        }

        /// <summary>
        /// Casts a convex shape against the collidable.
        /// </summary>
        /// <param name="castShape">Shape to cast.</param>
        /// <param name="startingTransform">Initial transform of the shape.</param>
        /// <param name="sweep">Sweep to apply to the shape.</param>
        /// <param name="hit">Hit data, if any.</param>
        /// <returns>Whether or not the cast hit anything.</returns>
        public override bool ConvexCast(ConvexShape castShape, ref RigidTransform startingTransform, ref Vector3 sweep, out RayHit hit)
        {
            RayCastResult result;
            bool toReturn = Shape.ConvexCast(castShape, ref startingTransform, ref sweep, out result);
            hit = result.HitData;
            return toReturn;
        }

        /// <summary>
        /// Casts a convex shape against the collidable.
        /// </summary>
        /// <param name="castShape">Shape to cast.</param>
        /// <param name="startingTransform">Initial transform of the shape.</param>
        /// <param name="sweep">Sweep to apply to the shape.</param>
        /// <param name="filter">Test to apply to the entry. If it returns true, the entry is processed, otherwise the entry is ignored. If a collidable hierarchy is present
        /// in the entry, this filter will be passed into inner ray casts.</param>
        /// <param name="hit">Hit data, if any.</param>
        /// <returns>Whether or not the cast hit anything.</returns>
        public override bool ConvexCast(ConvexShape castShape, ref RigidTransform startingTransform, ref Vector3 sweep, Func<BroadPhaseEntry, bool> filter, out RayHit hit)
        {
            RayCastResult result;
            bool toReturn = Shape.ConvexCast(castShape, ref startingTransform, ref sweep, filter, out result);
            hit = result.HitData;
            return toReturn;
        }



    }
}
