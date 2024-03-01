﻿using System;
using BEPUphysics.Entities;
using BEPUutilities.DataStructures;
using FixMath.NET;
using BEPUutilities;

namespace BEPUphysics.Constraints.SolverGroups
{
    /// <summary>
    /// Superclass of constraints that are composed of multiple subconstraints.
    /// </summary>
    public abstract class SolverGroup : SolverUpdateable
    {
        internal readonly RawList<SolverUpdateable> solverUpdateables = new RawList<SolverUpdateable>();


        /// <summary>
        /// Gets the solver updateables managed by this solver group.
        /// </summary>
        public ReadOnlyList<SolverUpdateable> SolverUpdateables
        {
            get
            {
                return new ReadOnlyList<SolverUpdateable>(solverUpdateables);
            }
        }




        /// <summary>
        /// Collects the entities which are affected by the solver group and updates the internal listing.
        /// </summary>
        protected internal override void CollectInvolvedEntities(RawList<Entity> outputInvolvedEntities)
        {
            foreach (SolverUpdateable item in solverUpdateables)
            {
                for (int i = 0; i < item.involvedEntities.Count; i++)
                {
                    if (!outputInvolvedEntities.Contains(item.involvedEntities.Elements[i]))
                    {
                        outputInvolvedEntities.Add(item.involvedEntities.Elements[i]);
                    }
                }
            }
        }


        /// <summary>
        /// Sets the activity state of the constraint based on the activity state of its connections.
        /// Called automatically by the space owning a constaint.  If a constraint is a sub-constraint that hasn't been directly added to the space,
        /// this may need to be called alongside the preStep from within the parent constraint.
        /// </summary>
        public override void UpdateSolverActivity()
        {
            if (isActive)
            {
                isActiveInSolver = false;
                for (int i = 0; i < solverUpdateables.Count; i++)
                {
                    var item = solverUpdateables.Elements[i];
                    item.UpdateSolverActivity();
                    isActiveInSolver |= item.isActiveInSolver;
                }
            }
            else
            {
                isActiveInSolver = false;
            }
        }

        protected void UpdateUpdateable(SolverUpdateable item, Fix64 dt)
        {
            item.SolverSettings.currentIterations = 0;
            item.SolverSettings.iterationsAtZeroImpulse = 0;
            if (item.isActiveInSolver)
                item.Update(dt);
        }

        protected void ExclusiveUpdateUpdateable(SolverUpdateable item)
        {
            if (item.isActiveInSolver)
                item.ExclusiveUpdate();
        }

        ///<summary>
        /// Performs the frame's configuration step.
        ///</summary>
        ///<param name="dt">Timestep duration.</param>
        public override void Update(Fix64 dt)
        {
            for (int i = 0; i < solverUpdateables.Count; i++)
            {
                UpdateUpdateable(solverUpdateables.Elements[i], dt);
            }
        }



        /// <summary>
        /// Performs any pre-solve iteration work that needs exclusive
        /// access to the members of the solver updateable.
        /// Usually, this is used for applying warmstarting impulses.
        /// </summary>
        public override void ExclusiveUpdate()
        {
            for (int i = 0; i < solverUpdateables.Count; i++)
            {
                ExclusiveUpdateUpdateable(solverUpdateables.Elements[i]);
            }
        }

        /// <summary>
        /// Solves a child updateable.  Some children may override the group's update method;
        /// this avoids code repeat.
        /// </summary>
        /// <param name="item"></param>
        /// <param name="activeConstraints"> </param>
        protected void SolveUpdateable(SolverUpdateable item, ref int activeConstraints)
        {
            if (item.isActiveInSolver)
            {
                SolverSettings subSolverSettings = item.solverSettings;

                subSolverSettings.currentIterations++;
                if (subSolverSettings.currentIterations <= solver.iterationLimit &&
                    subSolverSettings.currentIterations <= subSolverSettings.maximumIterationCount)
                {
                    if (item.SolveIteration() < subSolverSettings.minimumImpulse)
                    {
                        subSolverSettings.iterationsAtZeroImpulse++;
                        if (subSolverSettings.iterationsAtZeroImpulse > subSolverSettings.minimumIterationCount)
                            item.isActiveInSolver = false;
                        else
                        {
                            activeConstraints++;
                        }

                    }
                    else
                    {
                        subSolverSettings.iterationsAtZeroImpulse = 0;
                        activeConstraints++;
                    }
                }
                else
                {
                    item.isActiveInSolver = false;
                }

            }
        }

        /// <summary>
        /// Computes one iteration of the constraint to meet the solver updateable's goal.
        /// </summary>
        /// <returns>The rough applied impulse magnitude.</returns>
        public override Fix64 SolveIteration()
        {
            int activeConstraints = 0;
            for (int i = 0; i < solverUpdateables.Count; i++)
            {
                SolveUpdateable(solverUpdateables.Elements[i], ref activeConstraints);
            }
            isActiveInSolver = activeConstraints > 0;
            return solverSettings.minimumImpulse + F64.C1; //Never let the system deactivate due to low impulses; solver group takes care of itself.
        }


        /// <summary>
        /// Adds a solver updateable to the group.
        /// </summary>
        /// <param name="solverUpdateable">Solver updateable to add.</param>
        /// <exception cref="InvalidOperationException">Thrown when the SolverUpdateable to add to the SolverGroup already belongs to another SolverGroup or to a Space.</exception>
        protected void Add(SolverUpdateable solverUpdateable)
        {
            if (solverUpdateable.solver == null)
            {
                if (solverUpdateable.SolverGroup == null)
                {
                    solverUpdateables.Add(solverUpdateable);
                    solverUpdateable.SolverGroup = this;
                    solverUpdateable.Solver = solver;
                    OnInvolvedEntitiesChanged();
                }
                else
                {
                    throw new InvalidOperationException("Cannot add SolverUpdateable to SolverGroup; it already belongs to a SolverGroup.");
                }
            }
            else
            {
                throw new InvalidOperationException("Cannot add SolverUpdateable to SolverGroup; it already belongs to a solver.");
            }
        }

        /// <summary>
        /// Removes a solver updateable from the group.
        /// </summary>
        /// <param name="solverUpdateable">Solver updateable to remove.</param>
        /// <exception cref="InvalidOperationException">Thrown when the SolverUpdateable to remove from the SolverGroup doesn't actually belong to this SolverGroup.</exception>
        protected void Remove(SolverUpdateable solverUpdateable)
        {
            if (solverUpdateable.SolverGroup == this)
            {
                solverUpdateables.Remove(solverUpdateable);
                solverUpdateable.SolverGroup = null;
                solverUpdateable.Solver = null;
                OnInvolvedEntitiesChanged();
            }
            else
            {
                throw new InvalidOperationException("Cannot remove SolverUpdateable from SolverGroup; it doesn't belong to this SolverGroup.");
            }
        }

        /// <summary>
        /// Called after the object is added to a space.
        /// </summary>
        /// <param name="newSpace"></param>
        public override void OnAdditionToSpace(Space newSpace)
        {
            for (int i = 0; i < solverUpdateables.Count; i++)
            {
                solverUpdateables[i].OnAdditionToSpace(newSpace);
            }
        }

        /// <summary>
        /// Called before an object is removed from its space.
        /// </summary>
        public override void OnRemovalFromSpace(Space oldSpace)
        {
            for (int i = 0; i < solverUpdateables.Count; i++)
            {
                solverUpdateables[i].OnRemovalFromSpace(oldSpace);
            }
        }

        ///<summary>
        /// Called when the updateable is added to a solver.
        ///</summary>
        ///<param name="newSolver">Solver to which the updateable was added.</param>
        public override void OnAdditionToSolver(Solver newSolver)
        {
            for (int i = 0; i < solverUpdateables.Count; i++)
            {
                solverUpdateables[i].OnAdditionToSolver(newSolver);
            }
        }

        /// <summary>
        /// Called when the updateable is removed from its solver.
        /// </summary>
        /// <param name="oldSolver">Solver from which the updateable was removed.</param>
        public override void OnRemovalFromSolver(Solver oldSolver)
        {
            for (int i = 0; i < solverUpdateables.Count; i++)
            {
                solverUpdateables[i].OnRemovalFromSolver(oldSolver);
            }
        }

        ///<summary>
        /// Gets the solver to which the solver updateable belongs.
        ///</summary>
        public override Solver Solver
        {
            get
            {
                return solver;
            }
            protected internal set
            {
                base.Solver = value;
                for (int i = 0; i < solverUpdateables.Count; i++)
                {
                    solverUpdateables.Elements[i].Solver = value;
                }
            }
        }

    }
}