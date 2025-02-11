using System;
using System.Diagnostics;
using Unity.Mathematics;

namespace MotionBlending
{
    /// <summary>
    /// Inertialization is a type of blending between two poses.
    /// Typically to blend between two poses, we use crossfade. Both poses are stored queried during the transition
    /// and they are blended during the transition.
    /// The basic idea of inertialization is when we change the pose we change it for real but compute offsets that
    /// takes us from the new/target pose to the source one. Then, we decay this offset using a polynomial function or springs.
    /// Decaying the offset will progressively take us to the target pose.
    /// </summary>
    public class InertializationQuaternion
    {
        public quaternion[] InertializedQuaternions;
        public float3[] InertializedAngularVelocities;

        private readonly quaternion[] OffsetQuaternions;
        private readonly float3[] OffsetAngularVelocities;

        public InertializationQuaternion(int num)
        {
            InertializedQuaternions = new quaternion[num];
            InertializedAngularVelocities = new float3[num];
            OffsetQuaternions = new quaternion[num];
            for (int i = 0; i < num; i++) OffsetQuaternions[i] = quaternion.identity; // init to a valid quaternion
            OffsetAngularVelocities = new float3[num];
        }

        /// <summary>
        /// It sets up the inertialization, which can then by updated by calling Update(...).
        /// </summary>
        public void Transition(ReadOnlySpan<quaternion> sourceQuaternions, ReadOnlySpan<float3> sourceAngularVelocities,
                               ReadOnlySpan<quaternion> targetQuaternions, ReadOnlySpan<float3> targetAngularVelocities)
        {
            Debug.Assert(sourceQuaternions.Length == InertializedQuaternions.Length);
            Debug.Assert(sourceAngularVelocities.Length == InertializedQuaternions.Length);
            Debug.Assert(targetQuaternions.Length == InertializedQuaternions.Length);
            Debug.Assert(targetAngularVelocities.Length == InertializedQuaternions.Length);
            for (int i = 0; i < sourceQuaternions.Length; i++)
            {
                InertializeTransition(sourceQuaternions[i], sourceAngularVelocities[i],
                                      targetQuaternions[i], targetAngularVelocities[i],
                                      ref OffsetQuaternions[i], ref OffsetAngularVelocities[i]);
            }
        }

        /// <summary>
        /// Updates the inertialization decaying the offset from the
        /// source quaternions (specified in Transition(...)) to the target quaternions
        /// and applying it to the current target quaternions. 
        /// </summary>
        public void Update(ReadOnlySpan<quaternion> currentTargetQuaternions,
                           ReadOnlySpan<float3> currentTargetAngularVelocities,
                           float halfLife, float deltaTime)
        {
            Debug.Assert(currentTargetQuaternions.Length == InertializedQuaternions.Length);
            Debug.Assert(currentTargetAngularVelocities.Length == InertializedQuaternions.Length);
            for (int i = 0; i < currentTargetQuaternions.Length; i++)
            {
                InertializeUpdate(currentTargetQuaternions[i], currentTargetAngularVelocities[i],
                                  halfLife, deltaTime,
                                  ref OffsetQuaternions[i], ref OffsetAngularVelocities[i],
                                  out InertializedQuaternions[i], out InertializedAngularVelocities[i]);
            }
        }

        /// <summary>
        /// Compute the offsets from the source quaternions to the target quaternions.
        /// Offsets are in/out since we may start a inertialization in the middle of another inertialization.
        /// </summary>
        public static void InertializeTransition(quaternion sourceRot, float3 sourceAngularVel,
                                                 quaternion targetRot, float3 targetAngularVel,
                                                 ref quaternion offsetRot, ref float3 offsetAngularVel)
        {
            offsetRot = math.normalizesafe(Utils.Abs(math.mul(math.inverse(targetRot), math.mul(sourceRot, offsetRot))));
            offsetAngularVel = (sourceAngularVel + offsetAngularVel) - targetAngularVel;
        }

        /// <summary>
        /// Updates the inertialization decaying the offset and applying it to the target quaternions.
        /// </summary>
        public static void InertializeUpdate(quaternion targetRot, float3 targetAngularVel,
                                             float halfLife, float deltaTime,
                                             ref quaternion offsetRot, ref float3 offsetAngularVel,
                                             out quaternion newRot, out float3 newAngularVel)
        {
            DecaySpringDamperImplicit(ref offsetRot, ref offsetAngularVel, halfLife, deltaTime);
            newRot = math.mul(targetRot, offsetRot);
            newAngularVel = targetAngularVel + offsetAngularVel;
        }

        /// <summary>
        /// Special type of SpringDamperImplicit when the desired rotation is the identity.
        /// </summary>
        public static void DecaySpringDamperImplicit(ref quaternion rot, ref float3 angularVel, float halfLife, float deltaTime)
        {
            float y = Utils.HalfLifeToDamping(halfLife) / 2.0f; // this could be precomputed
            float3 j0 = rot.ToScaledAngleAxis();
            float3 j1 = angularVel + j0 * y;
            float eyedt = Utils.FastNEgeExp(y * deltaTime); // this could be precomputed if several agents use it the same frame

            rot = Utils.ToQuaternion(eyedt * (j0 + j1 * deltaTime));
            angularVel = eyedt * (angularVel - deltaTime * y * j1);
        }
    }
}