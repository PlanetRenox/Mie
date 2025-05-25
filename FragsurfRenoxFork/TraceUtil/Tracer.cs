using UnityEngine;

namespace Fragsurf.TraceUtil {
    public class Tracer {
        // Static values to avoid allocations
        private static readonly float normalRayOffset = 0.001f;
        private static readonly float normalRayDistance = 0.002f;

        /// <summary>
        /// Trace using any collider type
        /// </summary>
        /// <param name="collider">Collider to use for tracing</param>
        /// <param name="origin">Starting point</param>
        /// <param name="end">Ending point</param>
        /// <param name="layerMask">Layer mask for collision detection</param>
        /// <param name="colliderScale">Optional scale factor for the collider</param>
        /// <returns>Trace result</returns>
        public static Trace TraceCollider(Collider collider, Vector3 origin, Vector3 end, int layerMask, float colliderScale = 1f) {
            // Early exit if collider is null
            if (collider == null)
                return new Trace() { fraction = 1, startPos = origin, endPos = end };

            if (collider is BoxCollider) {
                // Box collider trace
                return TraceBox(origin, end, collider.bounds.extents, collider.contactOffset, layerMask, colliderScale);
            } else if (collider is CapsuleCollider) {
                // Capsule collider trace
                var capc = (CapsuleCollider)collider;

                Vector3 point1, point2;
                Movement.MovePhysics.GetCapsulePoints(capc, origin, out point1, out point2);

                return TraceCapsule(point1, point2, capc.radius, origin, end, capc.contactOffset, layerMask, colliderScale);
            }

            throw new System.NotImplementedException("Trace missing for collider: " + collider.GetType());
        }

        /// <summary>
        /// Trace using a capsule shape
        /// </summary>
        /// <param name="point1">Top point of capsule</param>
        /// <param name="point2">Bottom point of capsule</param>
        /// <param name="radius">Radius of capsule</param>
        /// <param name="start">Starting point of trace</param>
        /// <param name="destination">End point of trace</param>
        /// <param name="contactOffset">Contact offset for collision</param>
        /// <param name="layerMask">Layer mask for collision detection</param>
        /// <param name="colliderScale">Scale factor for the collider</param>
        /// <returns>Trace result</returns>
        public static Trace TraceCapsule(Vector3 point1, Vector3 point2, float radius, Vector3 start, Vector3 destination, float contactOffset, int layerMask, float colliderScale = 1f) {
            // Initialize result structure
            var result = new Trace() {
                startPos = start,
                endPos = destination
            };

            // Quick path - if start and destination are the same, no collision possible
            if ((destination - start).sqrMagnitude < 0.0001f) {
                result.fraction = 1;
                return result;
            }

            // Calculate direction and distance once
            Vector3 direction = (destination - start).normalized;
            float maxDistance = Vector3.Distance(start, destination);
            
            // Adjust for contact offset
            float longSide = contactOffset * 1.4142f; // sqrt(2) * contactOffset
            maxDistance += longSide;
            radius *= (1f - contactOffset);

            // Calculate scaled points
            Vector3 scaledPoint1 = point1 - Vector3.up * colliderScale * 0.5f;
            Vector3 scaledPoint2 = point2 + Vector3.up * colliderScale * 0.5f;
            float scaledRadius = radius * colliderScale;

            // Perform capsule cast
            RaycastHit hit;
            if (Physics.CapsuleCast(
                point1: scaledPoint1,
                point2: scaledPoint2,
                radius: scaledRadius,
                direction: direction,
                hitInfo: out hit,
                maxDistance: maxDistance,
                layerMask: layerMask,
                queryTriggerInteraction: QueryTriggerInteraction.Ignore)) {

                // Fill result data
                result.fraction = hit.distance / maxDistance;
                result.hitCollider = hit.collider;
                result.hitPoint = hit.point;
                result.planeNormal = hit.normal;
                result.distance = hit.distance;
                
                // Try to get a more accurate normal using a short raycast
                GetAccurateNormal(ref result, direction);
                
            } else {
                result.fraction = 1;
            }

            return result;
        }

        /// <summary>
        /// Trace using a box shape
        /// </summary>
        /// <param name="start">Starting point of trace</param>
        /// <param name="destination">End point of trace</param>
        /// <param name="extents">Box extents (half-size)</param>
        /// <param name="contactOffset">Contact offset for collision</param>
        /// <param name="layerMask">Layer mask for collision detection</param>
        /// <param name="colliderScale">Scale factor for the collider</param>
        /// <returns>Trace result</returns>
        public static Trace TraceBox(Vector3 start, Vector3 destination, Vector3 extents, float contactOffset, int layerMask, float colliderScale = 1f) {
            // Initialize result structure
            var result = new Trace() {
                startPos = start,
                endPos = destination
            };

            // Quick path - if start and destination are the same, no collision possible
            if ((destination - start).sqrMagnitude < 0.0001f) {
                result.fraction = 1;
                return result;
            }

            // Calculate direction and distance once
            Vector3 direction = (destination - start).normalized;
            float maxDistance = Vector3.Distance(start, destination);
            
            // Adjust for contact offset
            float longSide = contactOffset * 1.4142f; // sqrt(2) * contactOffset
            maxDistance += longSide;
            
            // Scale extents properly
            Vector3 scaledExtents = extents * (1f - contactOffset) * colliderScale;

            // Perform box cast
            RaycastHit hit;
            if (Physics.BoxCast(
                center: start,
                halfExtents: scaledExtents,
                direction: direction,
                orientation: Quaternion.identity,
                maxDistance: maxDistance,
                hitInfo: out hit,
                layerMask: layerMask,
                queryTriggerInteraction: QueryTriggerInteraction.Ignore)) {

                // Fill result data
                result.fraction = hit.distance / maxDistance;
                result.hitCollider = hit.collider;
                result.hitPoint = hit.point;
                result.planeNormal = hit.normal;
                result.distance = hit.distance;
                
                // Try to get a more accurate normal using a short raycast
                GetAccurateNormal(ref result, direction);
                
            } else {
                result.fraction = 1;
            }

            return result;
        }
        
        /// <summary>
        /// Attempt to get a more accurate surface normal using a short raycast
        /// </summary>
        /// <param name="result">Trace result to update</param>
        /// <param name="direction">Direction of the trace</param>
        private static void GetAccurateNormal(ref Trace result, Vector3 direction) {
            if (result.hitCollider != null) {
                RaycastHit normalHit;
                Ray normalRay = new Ray(result.hitPoint - direction * normalRayOffset, direction);
                if (result.hitCollider.Raycast(normalRay, out normalHit, normalRayDistance)) {
                    result.planeNormal = normalHit.normal;
                }
            }
        }
    }
}
