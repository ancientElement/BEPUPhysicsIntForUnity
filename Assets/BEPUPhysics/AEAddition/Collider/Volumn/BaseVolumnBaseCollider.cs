using System;
using AE_BEPUPhysics_Addition.Interface;
using BEPUphysics.Entities;
using UnityEngine;
using UnityEngine.Android;

namespace AE_BEPUPhysics_Addition
{
    [Serializable]
    public struct Constraints
    {
        public v3Bool FreezeRotation;
    }

    [Serializable]
    public struct v3Bool
    {
        public bool x;
        public bool y;
        public bool z;
    }

    public abstract class BaseVolumnBaseCollider : BaseCollider, IUpdatePosition
    {
        public bool IsStatic;
        public float Mass = 1;
        public Constraints Constraints;

        //子类的返回Entrity
        protected abstract Entity OnCreateEnity();

        //子类的返回Entrity
        protected abstract Entity GetEntity();

        //添加
        public override void AddIntoSpace(BEPUphysics.Space space)
        {
            var entity = OnCreateEnity();
            entity.Orientation = transform.rotation.ToFix64();
            space.Add(entity);
        }

        //移除
        public override void RemoveFromSpace(BEPUphysics.Space space)
        {
            space.Remove(GetEntity());
        }

        //设置速度
        public virtual void SetVeolicty(Vector3 velocity)
        {
            GetEntity().LinearVelocity = velocity.ToFix64();
            AEDebug.Log("速度" + velocity.ToFix64().ToString(), gameObject);
            AEDebug.Log("位置" + GetEntity().Position.ToString(), gameObject);
        }

        //获取速度
        public virtual Vector3 GetVelocity()
        {
            return GetEntity().LinearVelocity.ToFloat();
        }

        //设置位置
        public virtual void SetPosition(Vector3 pos)
        {
            GetEntity().Position = pos.ToFix64();
            AEDebug.Log("位置" + pos.ToFix64().ToString(), gameObject);
        }

        //获取位置
        public virtual Vector3 GetPosition()
        {
            return GetEntity().Position.ToFloat();
        }

        //锁定
        public virtual void Freeze()
        {
            var rotation = GetEntity().Orientation.ToFloat().eulerAngles;
            if (Constraints.FreezeRotation.x || Constraints.FreezeRotation.y || Constraints.FreezeRotation.z)
            {
                if (Constraints.FreezeRotation.x)
                {
                    rotation.x = 0;
                }

                if (Constraints.FreezeRotation.y)
                {
                    rotation.y = 0;
                }

                if (Constraints.FreezeRotation.z)
                {
                    rotation.z = 0;
                }
            }

            var quaternion = Quaternion.Euler(rotation);
            GetEntity().Orientation = quaternion.ToFix64();
        }

        //位置更新
        public void UpdatePosition()
        {
            UpdatePosition(GetEntity());
        }

        //位置更新
        public virtual void UpdatePosition(Entity entity)
        {
            if (IsStatic) return;
            transform.position = UnityEngine.Vector3.Lerp(transform.position, entity.Position.ToFloat(), 0.2f);
            transform.rotation = UnityEngine.Quaternion.Lerp(transform.rotation, entity.Orientation.ToFloat(), 0.2f);
            // AEDebug.Log("位置更新回调" +
            //             entity.Position.X + "..." +
            //             entity.Position.Y + "..." +
            //             entity.Position.Z + "..." +
            //             gameObject.name, gameObject);
        }
    }
}