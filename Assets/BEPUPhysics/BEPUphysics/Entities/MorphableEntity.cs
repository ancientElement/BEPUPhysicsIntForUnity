﻿using BEPUphysics.BroadPhaseEntries.MobileCollidables;
using BEPUutilities;
using BEPUphysics.CollisionShapes;
using FixMath.NET;

namespace BEPUphysics.Entities
{
    ///<summary>
    /// Entity with modifiable collision information.
    ///</summary>
    public class MorphableEntity : Entity
    {
        ///<summary>
        /// Gets or sets the collidable associated with the entity.
        ///</summary>
        public new EntityCollidable CollisionInformation
        {
            get
            {
                return base.CollisionInformation;
            }
            set
            {
                SetCollisionInformation(value);
            }
        }

        ///<summary>
        /// Constructs a new morphable entity.
        ///</summary>
        ///<param name="collisionInformation">Collidable to use with the entity.</param>
        public MorphableEntity(EntityCollidable collisionInformation)
            : base(collisionInformation)
        {
        }

        ///<summary>
        /// Constructs a new morphable entity.
        ///</summary>
        ///<param name="collisionInformation">Collidable to use with the entity.</param>
        ///<param name="mass">Mass of the entity.</param>
        public MorphableEntity(EntityCollidable collisionInformation, Fix64 mass)
            : base(collisionInformation, mass)
        {
        }

        ///<summary>
        /// Constructs a new morphable entity.
        ///</summary>
        ///<param name="collisionInformation">Collidable to use with the entity.</param>
        ///<param name="mass">Mass of the entity.</param>
        /// <param name="inertiaTensor">Inertia tensor of the entity.</param>
        public MorphableEntity(EntityCollidable collisionInformation, Fix64 mass, Matrix3x3 inertiaTensor)
            : base(collisionInformation, mass, inertiaTensor)
        {
        }


        ///<summary>
        /// Constructs a new morphable entity.
        ///</summary>
        ///<param name="shape">Shape to use with the entity.</param>
        public MorphableEntity(EntityShape shape)
            : base(shape)
        {
        }

        ///<summary>
        /// Constructs a new morphable entity.
        ///</summary>
        ///<param name="shape">Shape to use with the entity.</param>
        ///<param name="mass">Mass of the entity.</param>
        public MorphableEntity(EntityShape shape, Fix64 mass)
            : base(shape, mass)
        {
        }

        ///<summary>
        /// Constructs a new morphable entity.
        ///</summary>
        ///<param name="shape">Shape to use with the entity.</param>
        ///<param name="mass">Mass of the entity.</param>
        /// <param name="inertiaTensor">Inertia tensor of the entity.</param>
        public MorphableEntity(EntityShape shape, Fix64 mass, Matrix3x3 inertiaTensor)
            : base(shape, mass, inertiaTensor)
        {
        }


        /// <summary>
        /// Sets the collision information of the entity to another collidable.
        /// </summary>
        /// <param name="newCollisionInformation">New collidable to use.</param>
        public void SetCollisionInformation(EntityCollidable newCollisionInformation)
        {
            //Temporarily remove the object from the space.  
            //The reset process will update any systems that need to be updated.
            //This is not thread safe, but this operation should not be performed mid-frame anyway.
            Space space = Space;
            if (space != null)
                Space.Remove(this);

            CollisionInformation.Entity = null;

            if (isDynamic)
                Initialize(newCollisionInformation, mass);
            else
                Initialize(newCollisionInformation);

            if (space != null)
                space.Add(this);
        }

        /// <summary>
        /// Sets the collision information of the entity to another collidable.
        /// </summary>
        /// <param name="newCollisionInformation">New collidable to use.</param>
        /// <param name="newMass">New mass to use for the entity.</param>
        public void SetCollisionInformation(EntityCollidable newCollisionInformation, Fix64 newMass)
        {
            //Temporarily remove the object from the space.  
            //The reset process will update any systems that need to be updated.
            //This is not thread safe, but this operation should not be performed mid-frame anyway.
            Space space = Space;
            if (space != null)
                Space.Remove(this);

            CollisionInformation.Entity = null;

            Initialize(newCollisionInformation, newMass);

            if (space != null)
                space.Add(this);
        }

        /// <summary>
        /// Sets the collision information of the entity to another collidable.
        /// </summary>
        /// <param name="newCollisionInformation">New collidable to use.</param>
        /// <param name="newMass">New mass to use for the entity.</param>
        /// <param name="newInertia">New inertia tensor to use for the entity.</param>
        public void SetCollisionInformation(EntityCollidable newCollisionInformation, Fix64 newMass, Matrix3x3 newInertia)
        {
            //Temporarily remove the object from the space.  
            //The reset process will update any systems that need to be updated.
            //This is not thread safe, but this operation should not be performed mid-frame anyway.
            Space space = Space;
            if (space != null)
                Space.Remove(this);

            CollisionInformation.Entity = null;

            Initialize(newCollisionInformation, newMass, newInertia);

            if (space != null)
                space.Add(this);
        }


    }
}
