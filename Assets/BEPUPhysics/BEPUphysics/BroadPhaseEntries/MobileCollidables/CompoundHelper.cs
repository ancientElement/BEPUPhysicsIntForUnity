﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BEPUphysics.CollisionShapes;

using BEPUutilities;
using BEPUphysics.Entities;
using BEPUphysics.CollisionShapes.ConvexShapes;
using FixMath.NET;

namespace BEPUphysics.BroadPhaseEntries.MobileCollidables
{
    /// <summary>
    /// Contains methods to help with splitting compound objects into multiple pieces.
    /// </summary>
    public static class CompoundHelper
    {

        /// <summary>
        /// Splits a single compound collidable into two separate compound collidables and computes information needed by the simulation.
        /// </summary>
        /// <param name="splitPredicate">Delegate which determines if a child in the original compound should be moved to the new compound.</param>
        /// <param name="a">Original compound to be split.  Children in this compound will be removed and added to the other compound.</param>
        /// <param name="b">Compound to receive children removed from the original compound.</param>
        /// <returns>Whether or not the predicate returned true for any element in the original compound and split the compound.</returns>
        public static bool SplitCompound(Func<CompoundChild, bool> splitPredicate,
            Entity<CompoundCollidable> a, out Entity<CompoundCollidable> b)
        {

            ShapeDistributionInformation distributionInfoA, distributionInfoB;
            if (SplitCompound(splitPredicate, a, out b, out distributionInfoA, out distributionInfoB))
            {
                return true;
            }
            return false;
        }

        /// <summary>
        /// Splits a single compound collidable into two separate compound collidables and computes information needed by the simulation.
        /// </summary>
        /// <param name="splitPredicate">Delegate which determines if a child in the original compound should be moved to the new compound.</param>
        /// <param name="a">Original compound to be split.  Children in this compound will be removed and added to the other compound.</param>
        /// <param name="b">Compound to receive children removed from the original compound.</param>
        /// <param name="distributionInfoA">Volume, volume distribution, and center information about the new form of the original compound collidable.</param>
        /// <param name="distributionInfoB">Volume, volume distribution, and center information about the new compound collidable.</param>
        /// <returns>Whether or not the predicate returned true for any element in the original compound and split the compound.</returns>
        public static bool SplitCompound(Func<CompoundChild, bool> splitPredicate,
                        Entity<CompoundCollidable> a, out Entity<CompoundCollidable> b,
                        out ShapeDistributionInformation distributionInfoA, out ShapeDistributionInformation distributionInfoB)
        {
            var bCollidable = new CompoundCollidable { Shape = a.CollisionInformation.Shape };
            b = null;


            Fix64 weightA, weightB;
            if (SplitCompound(splitPredicate, a.CollisionInformation, bCollidable, out distributionInfoA, out distributionInfoB, out weightA, out weightB))
            {
                //Reconfigure the entities using the data computed in the split.
                Fix64 originalMass = a.mass;
                if (a.CollisionInformation.children.Count > 0)
                {
                    Fix64 newMassA = (weightA / (weightA + weightB)) * originalMass;
                    Matrix3x3.Multiply(ref distributionInfoA.VolumeDistribution, newMassA * InertiaHelper.InertiaTensorScale, out distributionInfoA.VolumeDistribution);
                    a.Initialize(a.CollisionInformation, newMassA, distributionInfoA.VolumeDistribution);
                }
                if (bCollidable.children.Count > 0)
                {
                    Fix64 newMassB = (weightB / (weightA + weightB)) * originalMass;
                    Matrix3x3.Multiply(ref distributionInfoB.VolumeDistribution, newMassB * InertiaHelper.InertiaTensorScale, out distributionInfoB.VolumeDistribution);
                    b = new Entity<CompoundCollidable>();
                    b.Initialize(bCollidable, newMassB, distributionInfoB.VolumeDistribution);
                }

                SplitReposition(a, b, ref distributionInfoA, ref distributionInfoB, weightA, weightB);
                return true;
            }
            return false;
        }

        static void SplitReposition(Entity a, Entity b, ref ShapeDistributionInformation distributionInfoA, ref ShapeDistributionInformation distributionInfoB, Fix64 weightA, Fix64 weightB)
        {
            //The compounds are not aligned with the original's position yet.
            //In order to align them, first look at the centers the split method computed.
            //They are offsets from the center of the original shape in local space.
            //These can be used to reposition the objects in world space.
            Vector3 weightedA, weightedB;
            Vector3.Multiply(ref distributionInfoA.Center, weightA, out weightedA);
            Vector3.Multiply(ref distributionInfoB.Center, weightB, out weightedB);
            Vector3 newLocalCenter;
            Vector3.Add(ref weightedA, ref weightedB, out newLocalCenter);
            Vector3.Divide(ref newLocalCenter, weightA + weightB, out newLocalCenter);

            Vector3 localOffsetA;
            Vector3 localOffsetB;
            Vector3.Subtract(ref distributionInfoA.Center, ref newLocalCenter, out localOffsetA);
            Vector3.Subtract(ref distributionInfoB.Center, ref newLocalCenter, out localOffsetB);

            Vector3 originalPosition = a.position;

            b.Orientation = a.Orientation;
            Vector3 offsetA = Quaternion.Transform(localOffsetA, a.Orientation);
            Vector3 offsetB = Quaternion.Transform(localOffsetB, a.Orientation);
            a.Position = originalPosition + offsetA;
            b.Position = originalPosition + offsetB;

            Vector3 originalLinearVelocity = a.linearVelocity;
            Vector3 originalAngularVelocity = a.angularVelocity;
            a.AngularVelocity = originalAngularVelocity;
            b.AngularVelocity = originalAngularVelocity;
            a.LinearVelocity = originalLinearVelocity + Vector3.Cross(originalAngularVelocity, offsetA);
            b.LinearVelocity = originalLinearVelocity + Vector3.Cross(originalAngularVelocity, offsetB);
        }


        /// <summary>
        /// Splits a single compound collidable into two separate compound collidables and computes information needed by the simulation.
        /// </summary>
        /// <param name="splitPredicate">Delegate which determines if a child in the original compound should be moved to the new compound.</param>
        /// <param name="a">Original compound to be split.  Children in this compound will be removed and added to the other compound.</param>
        /// <param name="b">Compound to receive children removed from the original compound.</param>
        /// <returns>Whether or not the predicate returned true for any element in the original compound and split the compound.</returns>
        public static bool SplitCompound(Func<CompoundChild, bool> splitPredicate,
            Entity<CompoundCollidable> a, Entity<CompoundCollidable> b)
        {
            ShapeDistributionInformation distributionInfoA, distributionInfoB;
            if (SplitCompound(splitPredicate, a, b, out distributionInfoA, out distributionInfoB))
            {
                return true;
            }
            return false;
        }

        /// <summary>
        /// Splits a single compound collidable into two separate compound collidables and computes information needed by the simulation.
        /// </summary>
        /// <param name="splitPredicate">Delegate which determines if a child in the original compound should be moved to the new compound.</param>
        /// <param name="distributionInfoA">Volume, volume distribution, and center information about the new form of the original compound collidable.</param>
        /// <param name="distributionInfoB">Volume, volume distribution, and center information about the new compound collidable.</param>
        /// <param name="a">Original compound to be split.  Children in this compound will be removed and added to the other compound.</param>
        /// <param name="b">Compound to receive children removed from the original compound.</param>
        /// <returns>Whether or not the predicate returned true for any element in the original compound and split the compound.</returns>
        public static bool SplitCompound(Func<CompoundChild, bool> splitPredicate,
                        Entity<CompoundCollidable> a, Entity<CompoundCollidable> b,
                        out ShapeDistributionInformation distributionInfoA, out ShapeDistributionInformation distributionInfoB)
        {
            Fix64 weightA, weightB;
            if (SplitCompound(splitPredicate, a.CollisionInformation, b.CollisionInformation, out distributionInfoA, out distributionInfoB, out weightA, out weightB))
            {
                //Reconfigure the entities using the data computed in the split.
                Fix64 originalMass = a.mass;
                if (a.CollisionInformation.children.Count > 0)
                {
                    Fix64 newMassA = (weightA / (weightA + weightB)) * originalMass;
                    Matrix3x3.Multiply(ref distributionInfoA.VolumeDistribution, newMassA * InertiaHelper.InertiaTensorScale, out distributionInfoA.VolumeDistribution);
                    a.Initialize(a.CollisionInformation, newMassA, distributionInfoA.VolumeDistribution);
                }

                if (b.CollisionInformation.children.Count > 0)
                {
                    Fix64 newMassB = (weightB / (weightA + weightB)) * originalMass;
                    Matrix3x3.Multiply(ref distributionInfoB.VolumeDistribution, newMassB * InertiaHelper.InertiaTensorScale, out distributionInfoB.VolumeDistribution);
                    b.Initialize(b.CollisionInformation, newMassB, distributionInfoB.VolumeDistribution);
                }

                SplitReposition(a, b, ref distributionInfoA, ref distributionInfoB, weightA, weightB);

                return true;
            }
            return false;
        }

        /// <summary>
        /// Splits a single compound collidable into two separate compound collidables and computes information needed by the simulation.
        /// </summary>
        /// <param name="splitPredicate">Delegate which determines if a child in the original compound should be moved to the new compound.</param>
        /// <param name="a">Original compound to be split.  Children in this compound will be removed and added to the other compound.</param>
        /// <param name="b">Compound to receive children removed from the original compound.</param>
        /// <param name="distributionInfoA">Volume, volume distribution, and center information about the new form of the original compound collidable.</param>
        /// <param name="distributionInfoB">Volume, volume distribution, and center information about the new compound collidable.</param>
        /// <param name="weightA">Total weight associated with the new form of the original compound collidable.</param>
        /// <param name="weightB">Total weight associated with the new compound collidable.</param>
        /// <returns>Whether or not the predicate returned true for any element in the original compound and split the compound.</returns>
        public static bool SplitCompound(Func<CompoundChild, bool> splitPredicate,
            CompoundCollidable a, CompoundCollidable b,
            out ShapeDistributionInformation distributionInfoA, out ShapeDistributionInformation distributionInfoB,
            out Fix64 weightA, out Fix64 weightB)
        {
            bool splitOccurred = false;
            for (int i = a.children.Count - 1; i >= 0; i--)
            {
                //The shape doesn't change during this process.  The entity could, though.
                //All of the other collidable information, like the Tag, CollisionRules, Events, etc. all stay the same.
                var child = a.children.Elements[i];
                if (splitPredicate(child))
                {
                    splitOccurred = true;

                    a.children.FastRemoveAt(i);
                    b.children.Add(child);
                    //The child event handler must be unhooked from the old compound and given to the new one.
                    child.CollisionInformation.events.Parent = b.Events;
                }
            }

            if (!splitOccurred)
            {
                //No split occurred, so we cannot proceed.
                distributionInfoA = new ShapeDistributionInformation();
                distributionInfoB = new ShapeDistributionInformation();
                weightA = F64.C0;
                weightB = F64.C0;
                return false;
            }

            //Compute the contributions from the original shape to the new form of the original collidable.
            distributionInfoA = new ShapeDistributionInformation();
            weightA = F64.C0;
            distributionInfoB = new ShapeDistributionInformation();
            weightB = F64.C0;
            for (int i = a.children.Count - 1; i >= 0; i--)
            {
                var child = a.children.Elements[i];
                var entry = child.Entry;
                Vector3 weightedCenter;
                Vector3.Multiply(ref entry.LocalTransform.Position, entry.Weight, out weightedCenter);
                Vector3.Add(ref weightedCenter, ref distributionInfoA.Center, out distributionInfoA.Center);
                distributionInfoA.Volume += entry.Shape.Volume;
                weightA += entry.Weight;
            }
            for (int i = b.children.Count - 1; i >= 0; i--)
            {
                var child = b.children.Elements[i];
                var entry = child.Entry;
                Vector3 weightedCenter;
                Vector3.Multiply(ref entry.LocalTransform.Position, entry.Weight, out weightedCenter);
                Vector3.Add(ref weightedCenter, ref distributionInfoB.Center, out distributionInfoB.Center);
                distributionInfoB.Volume += entry.Shape.Volume;
                weightB += entry.Weight;
            }

            //Average the center out.
            if (weightA > F64.C0)
                Vector3.Divide(ref distributionInfoA.Center, weightA, out distributionInfoA.Center);

            if (weightB > F64.C0)
                Vector3.Divide(ref distributionInfoB.Center, weightB, out distributionInfoB.Center);

            //Note that the 'entry' is from the Shape, and so the translations are local to the shape's center.
            //That is not technically the center of the new collidable- distributionInfoA.Center is.
            //Offset the child collidables by -distributionInfoA.Center using their local offset.
            Vector3 offsetA;
            Vector3.Negate(ref distributionInfoA.Center, out offsetA);
            Vector3 offsetB;
            Vector3.Negate(ref distributionInfoB.Center, out offsetB);

            //Compute the unscaled inertia tensor.
            for (int i = a.children.Count - 1; i >= 0; i--)
            {
                var child = a.children.Elements[i];
                var entry = child.Entry;
                Vector3 transformedOffset;
                Quaternion conjugate;
                Quaternion.Conjugate(ref entry.LocalTransform.Orientation, out conjugate);
                Quaternion.Transform(ref offsetA, ref conjugate, out transformedOffset);
                child.CollisionInformation.localPosition = transformedOffset;
                Matrix3x3 contribution;
                CompoundShape.TransformContribution(ref entry.LocalTransform, ref distributionInfoA.Center, ref entry.Shape.volumeDistribution, entry.Weight, out contribution);
                Matrix3x3.Add(ref contribution, ref distributionInfoA.VolumeDistribution, out distributionInfoA.VolumeDistribution);
            }
            for (int i = b.children.Count - 1; i >= 0; i--)
            {
                var child = b.children.Elements[i];
                var entry = child.Entry;
                Vector3 transformedOffset;
                Quaternion conjugate;
                Quaternion.Conjugate(ref entry.LocalTransform.Orientation, out conjugate);
                Quaternion.Transform(ref offsetB, ref conjugate, out transformedOffset);
                child.CollisionInformation.localPosition = transformedOffset;
                Matrix3x3 contribution;
                CompoundShape.TransformContribution(ref entry.LocalTransform, ref distributionInfoB.Center, ref entry.Shape.volumeDistribution, entry.Weight, out contribution);
                Matrix3x3.Add(ref contribution, ref distributionInfoB.VolumeDistribution, out distributionInfoB.VolumeDistribution);
            }

            //Normalize the volume distribution.
            Matrix3x3.Multiply(ref distributionInfoA.VolumeDistribution, F64.C1 / weightA, out distributionInfoA.VolumeDistribution);
            Matrix3x3.Multiply(ref distributionInfoB.VolumeDistribution, F64.C1 / weightB, out distributionInfoB.VolumeDistribution);

            //Update the hierarchies of the compounds.
            //TODO: Create a new method that does this quickly without garbage.  Requires a new Reconstruct method which takes a pool which stores the appropriate node types.
            a.hierarchy.Tree.Reconstruct(a.children);
            b.hierarchy.Tree.Reconstruct(b.children);

            return true;
        }


        static void RemoveReposition(Entity compound, ref ShapeDistributionInformation distributionInfo, Fix64 weight, Fix64 removedWeight, ref Vector3 removedCenter)
        {
            //The compounds are not aligned with the original's position yet.
            //In order to align them, first look at the centers the split method computed.
            //They are offsets from the center of the original shape in local space.
            //These can be used to reposition the objects in world space.
            Vector3 weightedA, weightedB;
            Vector3.Multiply(ref distributionInfo.Center, weight, out weightedA);
            Vector3.Multiply(ref removedCenter, removedWeight, out weightedB);
            Vector3 newLocalCenter;
            Vector3.Add(ref weightedA, ref weightedB, out newLocalCenter);
            Vector3.Divide(ref newLocalCenter, weight + removedWeight, out newLocalCenter);

            Vector3 localOffset;
            Vector3.Subtract(ref distributionInfo.Center, ref newLocalCenter, out localOffset);

            Vector3 originalPosition = compound.position;

            Vector3 offset = Quaternion.Transform(localOffset, compound.orientation);
            compound.Position = originalPosition + offset;

            Vector3 originalLinearVelocity = compound.linearVelocity;
            Vector3 originalAngularVelocity = compound.angularVelocity;
            compound.AngularVelocity = originalAngularVelocity;
            compound.LinearVelocity = originalLinearVelocity + Vector3.Cross(originalAngularVelocity, offset);
        }

        /// <summary>
        /// Removes a child from a compound body.
        /// </summary>
        /// <param name="childContributions">List of distribution information associated with each child shape of the whole compound shape used by the compound being split.</param>
        /// <param name="removalPredicate">Delegate which determines if a child in the original compound should be moved to the new compound.</param>
        /// <param name="compound">Original compound to have a child removed.</param>
        /// <returns>Whether or not the predicate returned true for any element in the original compound and split the compound.</returns>
        public static bool RemoveChildFromCompound(Entity<CompoundCollidable> compound, Func<CompoundChild, bool> removalPredicate, IList<ShapeDistributionInformation> childContributions)
        {
            ShapeDistributionInformation distributionInfo;
            if (RemoveChildFromCompound(compound, removalPredicate, childContributions, out distributionInfo))
            {
                return true;
            }
            else
                return false;
        }

        /// <summary>
        /// Removes a child from a compound body.
        /// </summary>
        /// <param name="childContributions">List of distribution information associated with each child shape of the whole compound shape used by the compound being split.</param>
        /// <param name="removalPredicate">Delegate which determines if a child in the original compound should be moved to the new compound.</param>
        /// <param name="distributionInfo">Volume, volume distribution, and center information about the new form of the original compound collidable.</param>
        /// <param name="compound">Original compound to have a child removed.</param>
        /// <returns>Whether or not the predicate returned true for any element in the original compound and split the compound.</returns>
        public static bool RemoveChildFromCompound(Entity<CompoundCollidable> compound, Func<CompoundChild, bool> removalPredicate, IList<ShapeDistributionInformation> childContributions,
                        out ShapeDistributionInformation distributionInfo)
        {
            Fix64 weight;
            Fix64 removedWeight;
            Vector3 removedCenter;
            if (RemoveChildFromCompound(compound.CollisionInformation, removalPredicate, childContributions, out distributionInfo, out weight, out removedWeight, out removedCenter))
            {
                //Reconfigure the entities using the data computed in the split.
                //Only bother if there are any children left in the compound!
                if (compound.CollisionInformation.Children.Count > 0)
                {
                    Fix64 originalMass = compound.mass;
                    Fix64 newMass = (weight / (weight + removedWeight)) * originalMass;
                    Matrix3x3.Multiply(ref distributionInfo.VolumeDistribution, newMass * InertiaHelper.InertiaTensorScale, out distributionInfo.VolumeDistribution);
                    compound.Initialize(compound.CollisionInformation, newMass, distributionInfo.VolumeDistribution);

                    RemoveReposition(compound, ref distributionInfo, weight, removedWeight, ref removedCenter);
                }

                return true;
            }
            else
                return false;
        }

        /// <summary>
        /// Removes a child from a compound collidable.
        /// </summary>
        /// <param name="compound">Compound collidable to remove a child from.</param>
        /// <param name="removalPredicate">Callback which analyzes a child and determines if it should be removed from the compound.</param>
        /// <param name="childContributions">Distribution contributions from all shapes in the compound shape.  This can include shapes which are not represented in the compound.</param>
        /// <param name="distributionInfo">Distribution information of the new compound.</param>
        /// <param name="weight">Total weight of the new compound.</param>
        /// <param name="removedWeight">Weight removed from the compound.</param>
        /// <param name="removedCenter">Center of the chunk removed from the compound.</param>
        /// <returns>Whether or not any removal took place.</returns>
        public static bool RemoveChildFromCompound(CompoundCollidable compound, Func<CompoundChild, bool> removalPredicate, IList<ShapeDistributionInformation> childContributions,
           out ShapeDistributionInformation distributionInfo, out Fix64 weight, out Fix64 removedWeight, out Vector3 removedCenter)
        {
            bool removalOccurred = false;
            removedWeight = F64.C0;
            removedCenter = new Vector3();
            for (int i = compound.children.Count - 1; i >= 0; i--)
            {
                //The shape doesn't change during this process.  The entity could, though.
                //All of the other collidable information, like the Tag, CollisionRules, Events, etc. all stay the same.
                var child = compound.children.Elements[i];
                if (removalPredicate(child))
                {
                    removalOccurred = true;
                    var entry = child.Entry;
                    removedWeight += entry.Weight;
                    Vector3 toAdd;
                    Vector3.Multiply(ref entry.LocalTransform.Position, entry.Weight, out toAdd);
                    Vector3.Add(ref removedCenter, ref toAdd, out removedCenter);
                    //The child event handler must be unhooked from the compound.
                    child.CollisionInformation.events.Parent = null;
                    compound.children.FastRemoveAt(i);
                }
            }

            if (!removalOccurred)
            {
                //No removal occurred, so we cannot proceed.
                distributionInfo = new ShapeDistributionInformation();
                weight = F64.C0;
                return false;
            }
            if (removedWeight > F64.C0)
            {
                Vector3.Divide(ref removedCenter, removedWeight, out removedCenter);
            }

            //Compute the contributions from the original shape to the new form of the original collidable.
            distributionInfo = new ShapeDistributionInformation();
            weight = F64.C0;
            for (int i = compound.children.Count - 1; i >= 0; i--)
            {
                var child = compound.children.Elements[i];
                var entry = child.Entry;
                var contribution = childContributions[child.shapeIndex];
                Vector3.Add(ref contribution.Center, ref entry.LocalTransform.Position, out contribution.Center);
                Vector3.Multiply(ref contribution.Center, child.Entry.Weight, out contribution.Center);
                Vector3.Add(ref contribution.Center, ref distributionInfo.Center, out distributionInfo.Center);
                distributionInfo.Volume += contribution.Volume;
                weight += entry.Weight;
            }
            //Average the center out.
            Vector3.Divide(ref distributionInfo.Center, weight, out distributionInfo.Center);

            //Note that the 'entry' is from the Shape, and so the translations are local to the shape's center.
            //That is not technically the center of the new collidable- distributionInfo.Center is.
            //Offset the child collidables by -distributionInfo.Center using their local offset.
            Vector3 offset;
            Vector3.Negate(ref distributionInfo.Center, out offset);

            //Compute the unscaled inertia tensor.
            for (int i = compound.children.Count - 1; i >= 0; i--)
            {
                var child = compound.children.Elements[i];
                var entry = child.Entry;
                Vector3 transformedOffset;
                Quaternion conjugate;
                Quaternion.Conjugate(ref entry.LocalTransform.Orientation, out conjugate);
                Quaternion.Transform(ref offset, ref conjugate, out transformedOffset);
                child.CollisionInformation.localPosition = transformedOffset;
                var contribution = childContributions[child.shapeIndex];
                CompoundShape.TransformContribution(ref entry.LocalTransform, ref distributionInfo.Center, ref contribution.VolumeDistribution, entry.Weight, out contribution.VolumeDistribution);
                //Vector3.Add(ref entry.LocalTransform.Position, ref offsetA, out entry.LocalTransform.Position);
                Matrix3x3.Add(ref contribution.VolumeDistribution, ref distributionInfo.VolumeDistribution, out distributionInfo.VolumeDistribution);
            }

            //Normalize the volume distribution.
            Matrix3x3.Multiply(ref distributionInfo.VolumeDistribution, F64.C1 / weight, out distributionInfo.VolumeDistribution);

            //Update the hierarchies of the compounds.
            //TODO: Create a new method that does this quickly without garbage.  Requires a new Reconstruct method which takes a pool which stores the appropriate node types.
            compound.hierarchy.Tree.Reconstruct(compound.children);

            return true;
        }

        /// <summary>
        /// Constructs a compound collidable containing only the specified subset of children.
        /// </summary>
        /// <param name="shape">Shape to base the compound collidable on.</param>
        /// <param name="childIndices">Indices of child shapes from the CompoundShape to include in the compound collidable.</param>
        /// <returns>Compound collidable containing only the specified subset of children.</returns>
        public static CompoundCollidable CreatePartialCompoundCollidable(CompoundShape shape, IList<int> childIndices)
        {
            if (childIndices.Count == 0)
                throw new ArgumentException("Cannot create a compound from zero shapes.");

            CompoundCollidable compound = new CompoundCollidable();
            Vector3 center = new Vector3();
            Fix64 totalWeight = F64.C0;
            for (int i = 0; i < childIndices.Count; i++)
            {
                //Create and add the child object itself.
                var entry = shape.shapes[childIndices[i]];
                compound.children.Add(new CompoundChild(shape, entry.Shape.GetCollidableInstance(), childIndices[i]));
                //Grab its entry to compute the center of mass of this subset.
                Vector3 toAdd;
                Vector3.Multiply(ref entry.LocalTransform.Position, entry.Weight, out toAdd);
                Vector3.Add(ref center, ref toAdd, out center);
                totalWeight += entry.Weight;
            }
            if (totalWeight <= F64.C0)
            {
                throw new ArgumentException("Compound has zero total weight; invalid configuration.");
            }
            Vector3.Divide(ref center, totalWeight, out center);
            //Our subset of the compound is not necessarily aligned with the shape's origin.
            //By default, an object will rotate around the center of the collision shape.
            //We can't modify the shape data itself since it could be shared, which leaves
            //modifying the local position of the collidable.
            //We have the subset position in shape space, so pull the collidable back into alignment
            //with the origin.
            //This approach matches the rest of the CompoundHelper's treatment of subsets.
            compound.LocalPosition = -center;

            //Recompute the hierarchy for the compound.
            compound.hierarchy.Tree.Reconstruct(compound.children);
            compound.Shape = shape;
            return compound;
        }

    }
}
