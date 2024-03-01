using System;
using System.Collections.Generic;
using BEPUphysics;
using BEPUutilities;
using FixMath.NET;

namespace AE_BEPUPhysics_Addition
{
    public class AEPhysicsMgr
    {
        private Space m_space;

        public AEPhysicsMgr(Vector3 grivaty)
        {
            m_space = new Space();
            m_space.ForceUpdater.Gravity = grivaty;
            m_space.TimeStepSettings.TimeStepDuration = 1 / 60m;
        }

        public void PhysicsUpdate(float delta)
        {
            m_space.Update((Fix64)delta);
        }

        //注册碰撞体
        public void RegisterColloder(AEBoxCollider collider)
        {
            collider.AddIntoSpace(m_space);
        }
    }
}