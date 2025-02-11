using System;
using System.Diagnostics;
using Unity.Mathematics;

namespace MotionBlending
{
    public class InertializationFloat3
    {
        public float3[] InertializedVectors;
        public float3[] InertializedVelocities;

        private readonly float3[] Offsets;
        private readonly float3[] OffsetVelocities;

        public InertializationFloat3(int num)
        {
            InertializedVectors = new float3[num];
            InertializedVelocities = new float3[num];
            Offsets = new float3[num];
            OffsetVelocities = new float3[num];
        }

        /// <summary>
        /// It sets up the inertialization, which can then by updated by calling Update(...).
        /// </summary>
        public void Transition(ReadOnlySpan<float3> source, ReadOnlySpan<float3> sourceVelocities,
                               ReadOnlySpan<float3> target, ReadOnlySpan<float3> targetVelocities)
        {
            Debug.Assert(source.Length == InertializedVectors.Length);
            Debug.Assert(sourceVelocities.Length == InertializedVectors.Length);
            Debug.Assert(target.Length == InertializedVectors.Length);
            Debug.Assert(targetVelocities.Length == InertializedVectors.Length);
            for (int i = 0; i < source.Length; i++)
            {
                InertializeTransition(source[i], sourceVelocities[i],
                                      target[i], targetVelocities[i],
                                      ref Offsets[i], ref OffsetVelocities[i]);
            }
        }

        /// <summary>
        /// Updates the inertialization decaying the offset from the
        /// source vectors (specified in Transition(...)) to the target vectors
        /// and applying it to the current target vectors. 
        /// </summary>
        public void Update(ReadOnlySpan<float3> currentTarget,
                           ReadOnlySpan<float3> currentTargetVelocities,
                           float halfLife, float deltaTime)
        {
            Debug.Assert(currentTarget.Length == InertializedVectors.Length);
            Debug.Assert(currentTargetVelocities.Length == InertializedVectors.Length);
            for (int i = 0; i < currentTarget.Length; i++)
            {
                InertializeUpdate(currentTarget[i], currentTargetVelocities[i],
                                  halfLife, deltaTime,
                                  ref Offsets[i], ref OffsetVelocities[i],
                                  out InertializedVectors[i], out InertializedVelocities[i]);
            }
        }

        /// <summary>
        /// Compute the offsets from the source vector to the target vector.
        /// Offsets are in/out since we may start a inertialization in the middle of another inertialization.
        /// </summary>
        public static void InertializeTransition(float3 source, float3 sourceVel,
                                                 float3 target, float3 targetVel,
                                                 ref float3 offset, ref float3 offsetVel)
        {
            offset = (source + offset) - target;
            offsetVel = (sourceVel + offsetVel) - targetVel;
        }

        /// <summary>
        /// Updates the inertialization decaying the offset and applying it to the target vectors.
        /// </summary>
        public static void InertializeUpdate(float3 target, float3 targetVel,
                                             float halfLife, float deltaTime,
                                             ref float3 offset, ref float3 offsetVel,
                                             out float3 newValue, out float3 newVel)
        {
            DecaySpringDamperImplicit(ref offset, ref offsetVel, halfLife, deltaTime);
            newValue = target + offset;
            newVel = targetVel + offsetVel;
        }

        /// <summary>
        /// Special type of SpringDamperImplicit when the desired vector is (0, 0, 0)
        /// </summary>
        public static void DecaySpringDamperImplicit(ref float3 pos, ref float3 velocity, float halfLife, float deltaTime)
        {
            float y = Utils.HalfLifeToDamping(halfLife) / 2.0f; // this could be precomputed
            float3 j1 = velocity + pos * y;
            float eyedt = Utils.FastNEgeExp(y * deltaTime); // this could be precomputed if several agents use it the same frame

            pos = eyedt * (pos + j1 * deltaTime);
            velocity = eyedt * (velocity - j1 * y * deltaTime);
        }
    }
}