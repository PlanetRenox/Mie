using UnityEngine;
using Fragsurf.TraceUtil;

namespace Fragsurf.Movement {
    public class MovePhysics {

        ///// Fields /////

        /// <summary>
        /// Change this if your ground is on a different layer
        /// </summary>
        public static int groundLayerMask = LayerMask.GetMask (new string[] { "Default", "Ground", "Player clip" }); //(1 << 0);

        private static readonly Collider[] _colliders = new Collider[maxCollisions];
        private static readonly Vector3[] _planes = new Vector3[maxClipPlanes];
        // Pre-cached vector to avoid allocation in hot paths
        private static readonly Vector3 horizontalPlane = new Vector3(1f, 0f, 1f);
        
        public const float HU2M = 52.4934383202f;
        private const int maxCollisions = 128;
        private const int maxClipPlanes = 5;
        private const int numBumps = 1;

        public const float MoveSlope = 0.7f;

        ///// Methods /////

        /// <summary>
        /// Resolves collisions between a collider and the environment
        /// </summary>
        /// <param name="collider">The collider to check</param>
        /// <param name="origin">The origin position, will be modified if collision occurs</param>
        /// <param name="velocity">The velocity, will be modified if collision occurs</param>
        /// <param name="rigidbodyPushForce">Force applied to rigidbodies on collision</param>
        /// <param name="velocityMultiplier">Multiplier for velocity changes</param>
        /// <param name="stepOffset">Maximum height of steps that can be climbed</param>
        /// <param name="mover">The move controllable entity</param>
        /// http://www.00jknight.com/blog/unity-character-controller
        
        public static void ResolveCollisions (Collider collider, ref Vector3 origin, ref Vector3 velocity, float rigidbodyPushForce, float velocityMultiplier = 1f, float stepOffset = 0f, IMoveControllable mover = null) {
            // Early exit if velocity is zero - no collision possible
            if (velocity.sqrMagnitude <= 0.0001f)
                return;

            // manual collision resolving
            int numOverlaps = 0;
            if (collider is CapsuleCollider) {
                var capc = collider as CapsuleCollider;

                Vector3 point1, point2;
                GetCapsulePoints (capc, origin, out point1, out point2);

                numOverlaps = Physics.OverlapCapsuleNonAlloc (point1, point2, capc.radius,
                    _colliders, groundLayerMask, QueryTriggerInteraction.Ignore);
            } else if (collider is BoxCollider) {
                numOverlaps = Physics.OverlapBoxNonAlloc (origin, collider.bounds.extents, _colliders,
                    Quaternion.identity, groundLayerMask, QueryTriggerInteraction.Ignore);
            }

            // Early exit if no overlaps
            if (numOverlaps == 0)
                return;

            // Pre-calculate horizontal velocity components once before the loop
            Vector3 forwardVelocity = Vector3.Scale(velocity, horizontalPlane);
            
            for (int i = 0; i < numOverlaps; i++) {
                if (_colliders[i] == null)
                    continue;

                Vector3 direction;
                float distance;

                if (Physics.ComputePenetration (collider, origin,
                    Quaternion.identity, _colliders[i], _colliders[i].transform.position,
                    _colliders[i].transform.rotation, out direction, out distance)) {
                    
                    // Step offset - only check if actually needed
                    if (stepOffset > 0f && mover != null && mover.moveData.useStepOffset) {
                        if (StepOffset (collider, _colliders[i], ref origin, ref velocity, rigidbodyPushForce, velocityMultiplier, stepOffset, direction, distance, forwardVelocity, mover))
                            return;
                    }

                    // Handle collision - normalize just once and cache results
                    if (direction.sqrMagnitude > 0.0001f) {
                        direction.Normalize();
                        
                        // Calculate these vectors once and reuse
                        Vector3 penetrationVector = direction * distance;
                        Vector3 velocityProjected = Vector3.Project(velocity, -direction);
                        velocityProjected.y = 0; // don't touch y velocity, we need it to calculate fall damage elsewhere
                        
                        // Apply collision response
                        origin += penetrationVector;
                        velocity -= velocityProjected * velocityMultiplier;

                        // Handle rigidbody interaction
                        Rigidbody rb = _colliders[i].GetComponentInParent<Rigidbody>();
                        if (rb != null && !rb.isKinematic)
                            rb.AddForceAtPosition(velocityProjected * velocityMultiplier * rigidbodyPushForce, origin, ForceMode.Impulse);
                    }
                }
            }
        }

        public static bool StepOffset (Collider collider, Collider otherCollider, ref Vector3 origin, ref Vector3 velocity, float rigidbodyPushForce, float velocityMultiplier, float stepOffset, Vector3 direction, float distance, Vector3 forwardVelocity, IMoveControllable mover) {
            // Return if step offset is 0
            if (stepOffset <= 0f)
                return false;

            // Get forward direction (return if we aren't moving/are only moving vertically)
            Vector3 forwardDirection = forwardVelocity.normalized;
            if (forwardDirection.sqrMagnitude < 0.0001f)
                return false;

            // Trace ground
            Trace groundTrace = Tracer.TraceCollider (collider, origin, origin + Vector3.down * 0.1f, groundLayerMask);
            if (groundTrace.hitCollider == null || Vector3.Angle (Vector3.up, groundTrace.planeNormal) > mover.moveData.slopeLimit)
                return false;

            // Cache slope limit check since it's reused later
            float slopeLimit = mover.moveData.slopeLimit;
            float stepOffsetValue = stepOffset; // Cache to avoid property access

            // Trace wall
            Trace wallTrace = Tracer.TraceCollider (collider, origin, origin + velocity, groundLayerMask, 0.9f);
            if (wallTrace.hitCollider == null || Vector3.Angle (Vector3.up, wallTrace.planeNormal) <= slopeLimit)
                return false;

            // Trace upwards (check for roof etc)
            float upDistance = stepOffsetValue;
            Trace upTrace = Tracer.TraceCollider (collider, origin, origin + Vector3.up * stepOffsetValue, groundLayerMask);
            if (upTrace.hitCollider != null)
                upDistance = upTrace.distance;

            // Don't bother doing the rest if we can't move up at all anyway
            if (upDistance <= 0f)
                return false;

            Vector3 upOrigin = origin + Vector3.up * upDistance;

            // Trace forwards (check for walls etc)
            float forwardMagnitude = stepOffsetValue;
            float forwardDistance = forwardMagnitude;
            Trace forwardTrace = Tracer.TraceCollider (collider, upOrigin, upOrigin + forwardDirection * Mathf.Max (0.2f, forwardMagnitude), groundLayerMask);
            if (forwardTrace.hitCollider != null)
                forwardDistance = forwardTrace.distance;
            
            // Don't bother doing the rest if we can't move forward anyway
            if (forwardDistance <= 0f)
                return false;

            Vector3 upForwardOrigin = upOrigin + forwardDirection * forwardDistance;

            // Trace down (find ground)
            float downDistance = upDistance;
            Trace downTrace = Tracer.TraceCollider (collider, upForwardOrigin, upForwardOrigin + Vector3.down * upDistance, groundLayerMask);
            if (downTrace.hitCollider != null)
                downDistance = downTrace.distance;

            // Check step size/angle
            float verticalStep = Mathf.Clamp (upDistance - downDistance, 0f, stepOffsetValue);
            float horizontalStep = forwardDistance;
            float stepAngle = Vector3.Angle (Vector3.forward, new Vector3 (0f, verticalStep, horizontalStep));
            if (stepAngle > slopeLimit)
                return false;

            // Get new position
            Vector3 endOrigin = origin + Vector3.up * verticalStep;
            
            // Actually move
            if (!Mathf.Approximately(origin.y, endOrigin.y) && forwardDistance > 0f) {
                origin = endOrigin + forwardDirection * forwardDistance * Time.deltaTime;
                return true;
            } 
            return false;
        }

        /// <summary>
        /// Apply friction to movement
        /// </summary>
        public static void Friction (ref Vector3 velocity, float stopSpeed, float friction, float deltaTime) {
            float speed = velocity.magnitude;

            // Early exit if speed is too low
            if (speed < 0.0001905f)
                return;

            float drop = 0f;

            // Apply ground friction
            float control = (speed < stopSpeed) ? stopSpeed : speed;
            drop += control * friction * deltaTime;

            // Scale the velocity
            float newspeed = speed - drop;
            if (newspeed < 0)
                newspeed = 0;

            // Apply velocity scaling
            if (!Mathf.Approximately(newspeed, speed)) {
                newspeed /= speed;
                velocity *= newspeed;
            }
        }

        /// <summary>
        /// Apply air acceleration to movement
        /// </summary>
        /// <param name="velocity">Current velocity vector</param>
        /// <param name="wishdir">Desired movement direction</param>
        /// <param name="wishspeed">Desired movement speed</param>
        /// <param name="accel">Acceleration factor</param>
        /// <param name="airCap">Max air speed</param>
        /// <param name="deltaTime">Time since last frame</param>
        /// <returns>Vector3 acceleration to add</returns>
        public static Vector3 AirAccelerate (Vector3 velocity, Vector3 wishdir, float wishspeed, float accel, float airCap, float deltaTime) {
            // Cap max speed
            float wishspd = Mathf.Min(wishspeed, airCap);
            
            // Calculate dot product once
            float currentspeed = Vector3.Dot(velocity, wishdir);

            // Calculate remaining speed
            float addspeed = wishspd - currentspeed;

            // If not adding any speed, return zero vector
            if (addspeed <= 0)
                return Vector3.zero;

            // Calculate acceleration
            float accelspeed = Mathf.Min(accel * wishspeed * deltaTime, addspeed);

            // Create result vector multiplied by wishdir
            return wishdir * accelspeed;
        }

        /// <summary>
        /// Accelerate the velocity in the desired direction
        /// </summary>
        /// <param name="currentVelocity">Current velocity vector</param>
        /// <param name="wishdir">Desired movement direction</param>
        /// <param name="wishspeed">Desired movement speed</param>
        /// <param name="accel">Acceleration factor</param>
        /// <param name="deltaTime">Time since last frame</param>
        /// <param name="surfaceFriction">Friction from the current surface</param>
        /// <returns>Vector3 acceleration to add</returns>
        public static Vector3 Accelerate (Vector3 currentVelocity, Vector3 wishdir, float wishspeed, float accel, float deltaTime, float surfaceFriction) {
            // Calculate dot product once
            float currentspeed = Vector3.Dot(currentVelocity, wishdir);

            // Calculate remaining speed
            float addspeed = wishspeed - currentspeed;

            // If not adding any speed, return zero vector
            if (addspeed <= 0)
                return Vector3.zero;

            // Calculate acceleration with friction factored in
            float accelspeed = Mathf.Min(accel * deltaTime * wishspeed * surfaceFriction, addspeed);

            // Return result vector multiplied by wishdir
            return wishdir * accelspeed;
        }

        /// <summary>
        /// Handle velocity reflection when colliding with surfaces
        /// </summary>
        /// <param name="velocity">Current velocity vector to be modified</param>
        /// <param name="collider">Collider to check collisions</param>
        /// <param name="origin">Current position</param>
        /// <param name="deltaTime">Time since last frame</param>
        /// <returns>Blocking flags</returns>
        public static int Reflect (ref Vector3 velocity, Collider collider, Vector3 origin, float deltaTime) {
            if (velocity.sqrMagnitude < 0.0001f) {
                return 0; // No need to reflect if not moving
            }

            Vector3 newVelocity = Vector3.zero;
            int blocked = 0;           // Assume not blocked
            int numplanes = 0;         // And not sliding along any planes
            Vector3 originalVelocity = velocity;  // Store original velocity
            Vector3 primalVelocity = velocity;

            float allFraction = 0f;
            float timeLeft = deltaTime;   // Total time for this movement operation.

            for (int bumpcount = 0; bumpcount < numBumps; bumpcount++) {
                if (velocity.sqrMagnitude < 0.0001f)
                    break;

                // Assume we can move all the way from the current origin to the destination
                Vector3 end = origin + velocity * timeLeft;
                Trace trace = Tracer.TraceCollider(collider, origin, end, groundLayerMask);

                allFraction += trace.fraction;

                if (trace.fraction > 0) {
                    // Actually covered some distance
                    originalVelocity = velocity;
                    numplanes = 0;
                }

                // If we covered the entire distance, we are done
                if (trace.fraction == 1)
                    break;

                // Check what kind of plane we hit
                float yNormal = trace.planeNormal.y;
                
                // If the plane has a high y component, it's probably a floor
                if (yNormal > MoveSlope)
                    blocked |= 1;      // Floor
                
                // If the plane has zero y component, it's a step or wall
                if (Mathf.Approximately(yNormal, 0))
                    blocked |= 2;      // Step/wall

                // Reduce time remaining by time used
                timeLeft -= timeLeft * trace.fraction;

                // Too many planes to clip against?
                if (numplanes >= maxClipPlanes) {
                    // This shouldn't normally happen
                    velocity = Vector3.zero;
                    break;
                }

                // Set up next clipping plane
                _planes[numplanes] = trace.planeNormal;
                numplanes++;

                // Only try reflection for first impact plane
                if (numplanes == 1) {
                    // If normal has slope, it's floor or slope
                    if (_planes[0].y > MoveSlope) {
                        return blocked;
                    } else {
                        // Otherwise clip velocity to reflect off the surface
                        ClipVelocity(originalVelocity, _planes[0], ref newVelocity, 1f);
                    }
                    
                    velocity = newVelocity;
                    originalVelocity = newVelocity;
                } else {
                    // Handle multiple reflection planes
                    for (int i = 0; i < numplanes; i++) {
                        ClipVelocity(originalVelocity, _planes[i], ref velocity, 1);

                        // Check if the clipped velocity goes through any other planes
                        bool valid = true;
                        for (int j = 0; j < numplanes && valid; j++) {
                            if (j != i) {
                                if (Vector3.Dot(velocity, _planes[j]) < 0)
                                    valid = false;
                            }
                        }

                        if (valid)
                            break;
                    }

                    // If we couldn't find a valid reflection solution
                    if (numplanes > 1) {
                        // Try to find a direction along the crease between planes
                        if (numplanes == 2) {
                            Vector3 dir = Vector3.Cross(_planes[0], _planes[1]).normalized;
                            float d = Vector3.Dot(dir, velocity);
                            velocity = dir * d;
                        } else {
                            velocity = Vector3.zero;
                            break;
                        }
                        
                        // If moving against original direction, stop
                        if (Vector3.Dot(velocity, primalVelocity) <= 0f) {
                            velocity = Vector3.zero;
                            break;
                        }
                    }
                }
            }

            // If we couldn't move at all, zero the velocity
            if (allFraction < 0.0001f)
                velocity = Vector3.zero;

            return blocked;
        }

        /// <summary>
        /// Clip velocity against a collision plane
        /// </summary>
        /// <param name="input">Input velocity</param>
        /// <param name="normal">Normal of the collision plane</param>
        /// <param name="output">Output clipped velocity</param>
        /// <param name="overbounce">Bounce factor</param>
        /// <returns>Blocking flags</returns>
        public static int ClipVelocity(Vector3 input, Vector3 normal, ref Vector3 output, float overbounce) {
            float angle = normal.y;
            int blocked = 0x00;     // Assume unblocked.

            if (angle > 0)          // Floor
                blocked |= 0x01;
                
            if (Mathf.Approximately(angle, 0))  // Wall/step
                blocked |= 0x02;
                
            // Calculate reflection using dot product
            float backoff = Vector3.Dot(input, normal) * overbounce;

            // Reflect velocity along the normal
            output = input - normal * backoff;

            // Ensure we're not still moving through the plane
            float adjust = Vector3.Dot(output, normal);
            if (adjust < 0.0f) {
                output -= normal * adjust;
            }

            return blocked;
        }

        /// <summary>
        /// Calculate the top and bottom points of a capsule collider
        /// </summary>
        /// <param name="capc">The capsule collider</param>
        /// <param name="origin">The origin position</param>
        /// <param name="p1">Output parameter for the top point</param>
        /// <param name="p2">Output parameter for the bottom point</param>
        public static void GetCapsulePoints(CapsuleCollider capc, Vector3 origin, out Vector3 p1, out Vector3 p2) {
            // Pre-calculate the distance to points
            float distanceToPoints = capc.height * 0.5f - capc.radius;
            
            // Calculate points directly using the cached value
            p1 = origin + capc.center + Vector3.up * distanceToPoints;
            p2 = origin + capc.center - Vector3.up * distanceToPoints;
        }
    }
}