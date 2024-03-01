using System;
using System.Collections.Generic;
using BEPUphysics.BroadPhaseEntries.MobileCollidables;
using BEPUphysics.EntityStateManagement;
 
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUutilities;
using BEPUutilities.DataStructures;
using FixMath.NET;

namespace BEPUphysics.Entities.Prefabs
{
    /// <summary>
    /// A shape formed from the convex hull around its subbodies.  Can collide and move.  After making an entity, add it to a Space so that the engine can manage it.
    /// </summary>
    public class WrappedBody : Entity<ConvexCollidable<WrappedShape>>
    {
        ///<summary>
        /// Gets the list of shapes in the wrapped body.
        ///</summary>
        public ObservableList<ConvexShapeEntry> Shapes
        {
            get
            {
                return CollisionInformation.Shape.Shapes;
            }
        }

        /// <exception cref="ArgumentException">Thrown when the subbodies list contains zero entities.</exception>
        private WrappedBody(IList<ConvexShapeEntry> subShapes, Fix64 mass)
        {
            Vector3 center;
            var shape = new WrappedShape(subShapes, out center);
            Initialize(new ConvexCollidable<WrappedShape>(shape), mass);
            Position = center;
        }

        /// <exception cref="ArgumentException">Thrown when the subbodies list contains zero entities.</exception>
        private WrappedBody(IList<ConvexShapeEntry> subShapes)
        {
            Vector3 center;
            var shape = new WrappedShape(subShapes, out center);
            Initialize(new ConvexCollidable<WrappedShape>(shape));
            Position = center;
        }

        /// <summary>
        /// Constructs a physically simulated box.
        /// </summary>
        /// <param name="position">Position of the box.</param>
        /// <param name="subBodies">List of entities composing the body.</param>
        /// <param name="mass">Mass of the object.</param>
        public WrappedBody(Vector3 position, IList<ConvexShapeEntry> subBodies, Fix64 mass)
            : this(subBodies, mass)
        {
            Position = position;
        }

        /// <summary>
        /// Constructs a nondynamic wrapped body.
        /// </summary>
        /// <param name="position">Position of the box.</param>
        /// <param name="subBodies">List of entities composing the body.</param>
        public WrappedBody(Vector3 position, IList<ConvexShapeEntry> subBodies)
            : this(subBodies)
        {
            Position = position;
        }

        /// <summary>
        /// Constructs a dynamic wrapped body.
        /// </summary>
        /// <param name="motionState">Motion state specifying the entity's initial state.</param>
        /// <param name="subBodies">List of entities composing the body.</param>
        /// <param name="mass">Mass of the object.</param>
        public WrappedBody(MotionState motionState, IList<ConvexShapeEntry> subBodies, Fix64 mass)
            : this(subBodies, mass)
        {
            MotionState = motionState;
        }

        /// <summary>
        /// Constructs a nondynamic wrapped body.
        /// </summary>
        /// <param name="motionState">Motion state specifying the entity's initial state.</param>
        /// <param name="subBodies">List of entities composing the body.</param>
        public WrappedBody(MotionState motionState, IList<ConvexShapeEntry> subBodies)
            : this(subBodies)
        {
            MotionState = motionState;
        }


      
    }
}