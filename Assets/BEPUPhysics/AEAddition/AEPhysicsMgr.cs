using System;
using System.Collections.Generic;
using AE_BEPUPhysics_Addition.Interface;
using BEPUphysics;
using BEPUphysics.Entities;
using BEPUutilities;
using FixMath.NET;

namespace AE_BEPUPhysics_Addition
{
    public class AEPhysicsMgr
    {
        private Space m_space;
        private Action m_updatePosition;
        private Action m_freezy;

        public AEPhysicsMgr(Vector3 grivaty)
        {
            m_space = new Space();
            m_space.ForceUpdater.Gravity = grivaty;
            m_space.TimeStepSettings.TimeStepDuration = 1 / 60m;
        }

        public void PhysicsUpdate(float dt)
        {
            m_freezy?.Invoke();
            m_space.Update(dt.ToFix64());
            //AEDebug.Log("物理帧更新");
        }

        public void PhysicsUpdate()
        {
            m_freezy?.Invoke();
            m_space.Update();
            //AEDebug.Log("物理帧更新");
        }

        public void UpdatePosition()
        {
            m_updatePosition?.Invoke();
        }

        //注册碰撞体
        public void RegisterCollider(BaseCollider baseCollider)
        {
            baseCollider.AddIntoSpace(m_space);
            if (baseCollider is IUpdatePosition)
            {
                m_updatePosition += (baseCollider as IUpdatePosition).UpdatePosition;
                m_freezy += (baseCollider as IUpdatePosition).Freeze;
            }
        }
    }
}