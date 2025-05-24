using UnityEngine;
using Fragsurf.TraceUtil;

namespace Fragsurf.Movement {
    public class MoveController {

        ///// Fields /////

        [HideInInspector] public Transform playerTransform;
        private IMoveControllable _mover;
        private MovementConfig _config;
        private float _deltaTime;

        public bool jumping = false;
        public bool crouching = false;
        public float speed = 0f;
        
        public Transform camera;
        public float cameraYPos = 0f;

        private float slideSpeedCurrent = 0f;
        private Vector3 slideDirection = Vector3.forward;

        private bool sliding = false;
        private bool wasSliding = false;
        private float slideDelay = 0f;
        
        private bool uncrouchDown = false;
        private float crouchLerp = 0f;

        private float frictionMult = 1f;

        ///// Methods /////

        Vector3 groundNormal = Vector3.up;

        /// <summary>
        /// 
        /// </summary>
        public void ProcessMovement (IMoveControllable mover, MovementConfig config, float deltaTime) {
            
            // cache instead of passing around parameters
            _mover = mover;
            _config = config;
            _deltaTime = deltaTime;
            
            if (_mover.moveData.laddersEnabled && !_mover.moveData.climbingLadder) {

                // Look for ladders
                LadderCheck (new Vector3(1f, 0.95f, 1f), _mover.moveData.velocity * Mathf.Clamp (_deltaTime * 2f, 0.025f, 0.25f));

            }
            
            if (_mover.moveData.laddersEnabled && _mover.moveData.climbingLadder) {
                
                LadderPhysics ();
                
            } else if (!_mover.moveData.underwater) {

                if (_mover.moveData.velocity.y <= 0f)
                    jumping = false;

                // apply gravity
                if (_mover.groundObject == null) {

                    _mover.moveData.velocity.y -= (_mover.moveData.gravityFactor * _config.gravity * _deltaTime);
                    _mover.moveData.velocity.y += _mover.baseVelocity.y * _deltaTime;

                }
                
                // input velocity, check for ground
                CheckGrounded ();
                CalculateMovementVelocity ();
                
            } else {

                // Do underwater logic
                UnderwaterPhysics ();

            }

            float yVel = _mover.moveData.velocity.y;
            _mover.moveData.velocity.y = 0f;
            _mover.moveData.velocity = Vector3.ClampMagnitude (_mover.moveData.velocity, _config.maxVelocity);
            speed =  _mover.moveData.velocity.magnitude;
            _mover.moveData.velocity.y = yVel;
            
            if (_mover.moveData.velocity.sqrMagnitude == 0f) {

                // Do collisions while standing still
                MovePhysics.ResolveCollisions (_mover.collider, ref _mover.moveData.origin, ref _mover.moveData.velocity, _mover.moveData.rigidbodyPushForce, 1f, _mover.moveData.stepOffset, _mover);

            } else {

                float maxDistPerFrame = 0.2f;
                Vector3 velocityThisFrame = _mover.moveData.velocity * _deltaTime;
                float velocityDistLeft = velocityThisFrame.magnitude;
                float initialVel = velocityDistLeft;
                while (velocityDistLeft > 0f) {

                    float amountThisLoop = Mathf.Min (maxDistPerFrame, velocityDistLeft);
                    velocityDistLeft -= amountThisLoop;

                    // increment origin
                    Vector3 velThisLoop = velocityThisFrame * (amountThisLoop / initialVel);
                    _mover.moveData.origin += velThisLoop;

                    // don't penetrate walls
                    MovePhysics.ResolveCollisions (_mover.collider, ref _mover.moveData.origin, ref _mover.moveData.velocity, _mover.moveData.rigidbodyPushForce, amountThisLoop / initialVel, _mover.moveData.stepOffset, _mover);

                }

            }

            _mover.moveData.groundedTemp = _mover.moveData.grounded;

            _mover = null;
            
        }

        /// <summary>
        /// 
        /// </summary>
        private void CalculateMovementVelocity () {
            switch (_mover.moveType) {

                case MoveType.Walk:

                if (_mover.groundObject == null) {

                    /*
                    // AIR MOVEMENT
                    */

                    wasSliding = false;

                    // apply movement from input
                    _mover.moveData.velocity += AirInputMovement ();

                    // let the magic happen
                    MovePhysics.Reflect (ref _mover.moveData.velocity, _mover.collider, _mover.moveData.origin, _deltaTime);

                } else {

                    /*
                    //  GROUND MOVEMENT
                    */

                    // Sliding
                    if (!wasSliding) {

                        slideDirection = new Vector3 (_mover.moveData.velocity.x, 0f, _mover.moveData.velocity.z).normalized;
                        slideSpeedCurrent = Mathf.Max (_config.maximumSlideSpeed, new Vector3 (_mover.moveData.velocity.x, 0f, _mover.moveData.velocity.z).magnitude);

                    }

                    sliding = false;
                    if (_mover.moveData.velocity.magnitude > _config.minimumSlideSpeed && _mover.moveData.slidingEnabled && _mover.moveData.crouching && slideDelay <= 0f) {

                        if (!wasSliding)
                            slideSpeedCurrent = Mathf.Clamp (slideSpeedCurrent * _config.slideSpeedMultiplier, _config.minimumSlideSpeed, _config.maximumSlideSpeed);

                        sliding = true;
                        wasSliding = true;
                        SlideMovement ();
                        return;

                    } else {

                        if (slideDelay > 0f)
                            slideDelay -= _deltaTime;

                        if (wasSliding)
                            slideDelay = _config.slideDelay;

                        wasSliding = false;

                    }
                    
                    float fric = crouching ? _config.crouchFriction : _config.friction;
                    float accel = crouching ? _config.crouchAcceleration : _config.acceleration;
                    float decel = crouching ? _config.crouchDeceleration : _config.deceleration;
                    
                    // Get movement directions
                    Vector3 forward = Vector3.Cross (groundNormal, -playerTransform.right);
                    Vector3 right = Vector3.Cross (groundNormal, forward);

                    float speed = _mover.moveData.sprinting ? _config.sprintSpeed : _config.walkSpeed;
                    if (crouching)
                        speed = _config.crouchSpeed;

                    Vector3 _wishDir;

                    // Jump and friction
                    if (_mover.moveData.wishJump) {

                        ApplyFriction (0.0f, true, true);
                        Jump ();
                        return;

                    } else {

                        ApplyFriction (1.0f * frictionMult, true, true);

                    }

                    float forwardMove = _mover.moveData.verticalAxis;
                    float rightMove = _mover.moveData.horizontalAxis;

                    _wishDir = forwardMove * forward + rightMove * right;
                    _wishDir.Normalize ();
                    Vector3 moveDirNorm = _wishDir;

                    Vector3 forwardVelocity = Vector3.Cross (groundNormal, Quaternion.AngleAxis (-90, Vector3.up) * new Vector3 (_mover.moveData.velocity.x, 0f, _mover.moveData.velocity.z));

                    // Set the target speed of the player
                    float _wishSpeed = _wishDir.magnitude;
                    _wishSpeed *= speed;

                    // Accelerate
                    float yVel = _mover.moveData.velocity.y;
                    Accelerate (_wishDir, _wishSpeed, accel * Mathf.Min (frictionMult, 1f), false);

                    float maxVelocityMagnitude = _config.maxVelocity;
                    _mover.moveData.velocity = Vector3.ClampMagnitude (new Vector3 (_mover.moveData.velocity.x, 0f, _mover.moveData.velocity.z), maxVelocityMagnitude);
                    _mover.moveData.velocity.y = yVel;

                    // Calculate how much slopes should affect movement
                    float yVelocityNew = forwardVelocity.normalized.y * new Vector3 (_mover.moveData.velocity.x, 0f, _mover.moveData.velocity.z).magnitude;

                    // Apply the Y-movement from slopes
                    _mover.moveData.velocity.y = yVelocityNew * (_wishDir.y < 0f ? 1.2f : 1.0f);
                    float removableYVelocity = _mover.moveData.velocity.y - yVelocityNew;

                }

                break;

            } // END OF SWITCH STATEMENT
        }

        private void UnderwaterPhysics () {

            _mover.moveData.velocity = Vector3.Lerp (_mover.moveData.velocity, Vector3.zero, _config.underwaterVelocityDampening * _deltaTime);

            // Gravity
            if (!CheckGrounded ())
                _mover.moveData.velocity.y -= _config.underwaterGravity * _deltaTime;

            // Swimming upwards
            if (Input.GetButton ("Jump"))
                _mover.moveData.velocity.y += _config.swimUpSpeed * _deltaTime;

            float fric = _config.underwaterFriction;
            float accel = _config.underwaterAcceleration;
            float decel = _config.underwaterDeceleration;

            ApplyFriction (1f, true, false);

            // Get movement directions
            Vector3 forward = Vector3.Cross (groundNormal, -playerTransform.right);
            Vector3 right = Vector3.Cross (groundNormal, forward);

            float speed = _config.underwaterSwimSpeed;

            Vector3 _wishDir;

            float forwardMove = _mover.moveData.verticalAxis;
            float rightMove = _mover.moveData.horizontalAxis;

            _wishDir = forwardMove * forward + rightMove * right;
            _wishDir.Normalize ();
            Vector3 moveDirNorm = _wishDir;

            Vector3 forwardVelocity = Vector3.Cross (groundNormal, Quaternion.AngleAxis (-90, Vector3.up) * new Vector3 (_mover.moveData.velocity.x, 0f, _mover.moveData.velocity.z));

            // Set the target speed of the player
            float _wishSpeed = _wishDir.magnitude;
            _wishSpeed *= speed;

            // Accelerate
            float yVel = _mover.moveData.velocity.y;
            Accelerate (_wishDir, _wishSpeed, accel, false);

            float maxVelocityMagnitude = _config.maxVelocity;
            _mover.moveData.velocity = Vector3.ClampMagnitude (new Vector3 (_mover.moveData.velocity.x, 0f, _mover.moveData.velocity.z), maxVelocityMagnitude);
            _mover.moveData.velocity.y = yVel;

            float yVelStored = _mover.moveData.velocity.y;
            _mover.moveData.velocity.y = 0f;

            // Calculate how much slopes should affect movement
            float yVelocityNew = forwardVelocity.normalized.y * new Vector3 (_mover.moveData.velocity.x, 0f, _mover.moveData.velocity.z).magnitude;

            // Apply the Y-movement from slopes
            _mover.moveData.velocity.y = Mathf.Min (Mathf.Max (0f, yVelocityNew) + yVelStored, speed);

            // Jumping out of water
            bool movingForwards = playerTransform.InverseTransformVector (_mover.moveData.velocity).z > 0f;
            Trace waterJumpTrace = TraceBounds (playerTransform.position, playerTransform.position + playerTransform.forward * 0.1f, MovePhysics.groundLayerMask);
            if (waterJumpTrace.hitCollider != null && Vector3.Angle (Vector3.up, waterJumpTrace.planeNormal) >= _config.slopeLimit && Input.GetButton ("Jump") && !_mover.moveData.cameraUnderwater && movingForwards)
                _mover.moveData.velocity.y = Mathf.Max (_mover.moveData.velocity.y, _config.jumpForce);

        }
        
        private void LadderCheck (Vector3 colliderScale, Vector3 direction) {

            if (_mover.moveData.velocity.sqrMagnitude <= 0f)
                return;
            
            bool foundLadder = false;

            RaycastHit [] hits = Physics.BoxCastAll (_mover.moveData.origin, Vector3.Scale (_mover.collider.bounds.size * 0.5f, colliderScale), Vector3.Scale (direction, new Vector3 (1f, 0f, 1f)), Quaternion.identity, direction.magnitude, MovePhysics.groundLayerMask, QueryTriggerInteraction.Collide);
            foreach (RaycastHit hit in hits) {

                Ladder ladder = hit.transform.GetComponentInParent<Ladder> ();
                if (ladder != null) {

                    bool allowClimb = true;
                    float ladderAngle = Vector3.Angle (Vector3.up, hit.normal);
                    if (_mover.moveData.angledLaddersEnabled) {

                        if (hit.normal.y < 0f)
                            allowClimb = false;
                        else {
                            
                            if (ladderAngle <= _mover.moveData.slopeLimit)
                                allowClimb = false;

                        }

                    } else if (hit.normal.y != 0f)
                        allowClimb = false;

                    if (allowClimb) {
                        foundLadder = true;
                        if (_mover.moveData.climbingLadder == false) {

                            _mover.moveData.climbingLadder = true;
                            _mover.moveData.ladderNormal = hit.normal;
                            _mover.moveData.ladderDirection = -hit.normal * direction.magnitude * 2f;

                            if (_mover.moveData.angledLaddersEnabled) {

                                Vector3 sideDir = hit.normal;
                                sideDir.y = 0f;
                                sideDir = Quaternion.AngleAxis (-90f, Vector3.up) * sideDir;

                                _mover.moveData.ladderClimbDir = Quaternion.AngleAxis (90f, sideDir) * hit.normal;
                                _mover.moveData.ladderClimbDir *= 1f/ _mover.moveData.ladderClimbDir.y; // Make sure Y is always 1

                            } else
                                _mover.moveData.ladderClimbDir = Vector3.up;
                            
                        }
                        
                    }

                }

            }

            if (!foundLadder) {
                
                _mover.moveData.ladderNormal = Vector3.zero;
                _mover.moveData.ladderVelocity = Vector3.zero;
                _mover.moveData.climbingLadder = false;
                _mover.moveData.ladderClimbDir = Vector3.up;

            }

        }

        private void LadderPhysics () {
            
            _mover.moveData.ladderVelocity = _mover.moveData.ladderClimbDir * _mover.moveData.verticalAxis * 6f;

            _mover.moveData.velocity = Vector3.Lerp (_mover.moveData.velocity, _mover.moveData.ladderVelocity, _deltaTime * 10f);

            LadderCheck (Vector3.one, _mover.moveData.ladderDirection);
            
            Trace floorTrace = TraceToFloor ();
            if (_mover.moveData.verticalAxis < 0f && floorTrace.hitCollider != null && Vector3.Angle (Vector3.up, floorTrace.planeNormal) <= _mover.moveData.slopeLimit) {

                _mover.moveData.velocity = _mover.moveData.ladderNormal * 0.5f;
                _mover.moveData.ladderVelocity = Vector3.zero;
                _mover.moveData.climbingLadder = false;

            }

            if (_mover.moveData.wishJump) {

                _mover.moveData.velocity = _mover.moveData.ladderNormal * 4f;
                _mover.moveData.ladderVelocity = Vector3.zero;
                _mover.moveData.climbingLadder = false;
                
            }
            
        }
        
        private void Accelerate (Vector3 wishDir, float wishSpeed, float acceleration, bool yMovement) {

            // Initialise variables
            float _addSpeed;
            float _accelerationSpeed;
            float _currentSpeed;
            
            // again, no idea
            _currentSpeed = Vector3.Dot (_mover.moveData.velocity, wishDir);
            _addSpeed = wishSpeed - _currentSpeed;

            // If you're not actually increasing your speed, stop here.
            if (_addSpeed <= 0)
                return;

            // won't bother trying to understand any of this, really
            _accelerationSpeed = Mathf.Min (acceleration * _deltaTime * wishSpeed, _addSpeed);

            // Add the velocity.
            _mover.moveData.velocity.x += _accelerationSpeed * wishDir.x;
            if (yMovement) { _mover.moveData.velocity.y += _accelerationSpeed * wishDir.y; }
            _mover.moveData.velocity.z += _accelerationSpeed * wishDir.z;

        }

        private void ApplyFriction (float t, bool yAffected, bool grounded) {

            // Initialise variables
            Vector3 _vel = _mover.moveData.velocity;
            float _speed;
            float _newSpeed;
            float _control;
            float _drop;

            // Set Y to 0, speed to the magnitude of movement and drop to 0. I think drop is the amount of speed that is lost, but I just stole this from the internet, idk.
            _vel.y = 0.0f;
            _speed = _vel.magnitude;
            _drop = 0.0f;

            float fric = crouching ? _config.crouchFriction : _config.friction;
            float accel = crouching ? _config.crouchAcceleration : _config.acceleration;
            float decel = crouching ? _config.crouchDeceleration : _config.deceleration;

            // Only apply friction if the player is grounded
            if (grounded) {
                
                // i honestly have no idea what this does tbh
                _vel.y = _mover.moveData.velocity.y;
                _control = _speed < decel ? decel : _speed;
                _drop = _control * fric * _deltaTime * t;

            }

            // again, no idea, but comments look cool
            _newSpeed = Mathf.Max (_speed - _drop, 0f);
            if (_speed > 0.0f)
                _newSpeed /= _speed;

            // Set the end-velocity
            _mover.moveData.velocity.x *= _newSpeed;
            if (yAffected == true) { _mover.moveData.velocity.y *= _newSpeed; }
            _mover.moveData.velocity.z *= _newSpeed;

        }

        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        private Vector3 AirInputMovement () {

            Vector3 wishVel, wishDir;
            float wishSpeed;

            GetWishValues (out wishVel, out wishDir, out wishSpeed);

            if (_config.clampAirSpeed && (wishSpeed != 0f && (wishSpeed > _config.maxSpeed))) {

                wishVel = wishVel * (_config.maxSpeed / wishSpeed);
                wishSpeed = _config.maxSpeed;

            }

            return MovePhysics.AirAccelerate (_mover.moveData.velocity, wishDir, wishSpeed, _config.airAcceleration, _config.airCap, _deltaTime);

        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="wishVel"></param>
        /// <param name="wishDir"></param>
        /// <param name="wishSpeed"></param>
        private void GetWishValues (out Vector3 wishVel, out Vector3 wishDir, out float wishSpeed) {

            wishVel = Vector3.zero;
            wishDir = Vector3.zero;
            wishSpeed = 0f;

            Vector3 forward = _mover.forward,
                right = _mover.right;

            forward [1] = 0;
            right [1] = 0;
            forward.Normalize ();
            right.Normalize ();

            for (int i = 0; i < 3; i++)
                wishVel [i] = forward [i] * _mover.moveData.forwardMove + right [i] * _mover.moveData.sideMove;
            wishVel [1] = 0;

            wishSpeed = wishVel.magnitude;
            wishDir = wishVel.normalized;

        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="velocity"></param>
        /// <param name="jumpPower"></param>
        private void Jump () {
            
            if (!_config.autoBhop)
                _mover.moveData.wishJump = false;
            
            _mover.moveData.velocity.y += _config.jumpForce;
            jumping = true;

        }

        /// <summary>
        /// 
        /// </summary>
        private bool CheckGrounded () {

            _mover.moveData.surfaceFriction = 1f;
            var movingUp = _mover.moveData.velocity.y > 0f;
            var trace = TraceToFloor ();

            float groundSteepness = Vector3.Angle (Vector3.up, trace.planeNormal);

            if (trace.hitCollider == null || groundSteepness > _config.slopeLimit || (jumping && _mover.moveData.velocity.y > 0f)) {

                SetGround (null);

                if (movingUp && _mover.moveType != MoveType.Noclip)
                    _mover.moveData.surfaceFriction = _config.airFriction;
                
                return false;

            } else {

                groundNormal = trace.planeNormal;
                SetGround (trace.hitCollider.gameObject);
                return true;

            }

        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="obj"></param>
        private void SetGround (GameObject obj) {

            if (obj != null) {

                _mover.groundObject = obj;
                _mover.moveData.velocity.y = 0;

            } else
                _mover.groundObject = null;

        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="start"></param>
        /// <param name="end"></param>
        /// <param name="layerMask"></param>
        /// <returns></returns>
        private Trace TraceBounds (Vector3 start, Vector3 end, int layerMask) {

            return Tracer.TraceCollider (_mover.collider, start, end, layerMask);

        }

        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        private Trace TraceToFloor () {

            var down = _mover.moveData.origin;
            down.y -= 0.15f;

            return Tracer.TraceCollider (_mover.collider, _mover.moveData.origin, down, MovePhysics.groundLayerMask);

        }

        public void Crouch (IMoveControllable mover, MovementConfig config, float deltaTime) {

            _mover = mover;
            _config = config;
            _deltaTime = deltaTime;

            if (_mover == null)
                return;

            if (_mover.collider == null)
                return;

            bool grounded = _mover.groundObject != null;
            bool wantsToCrouch = _mover.moveData.crouching;

            float crouchingHeight = Mathf.Clamp (_mover.moveData.crouchingHeight, 0.01f, 1f);
            float heightDifference = _mover.moveData.defaultHeight - _mover.moveData.defaultHeight * crouchingHeight;

            if (grounded)
                uncrouchDown = false;

            // Crouching input
            if (grounded)
                crouchLerp = Mathf.Lerp (crouchLerp, wantsToCrouch ? 1f : 0f, _deltaTime * _mover.moveData.crouchingSpeed);
            else if (!grounded && !wantsToCrouch && crouchLerp < 0.95f)
                crouchLerp = 0f;
            else if (!grounded && wantsToCrouch)
                crouchLerp = 1f;

            // Collider and position changing
            if (crouchLerp > 0.9f && !crouching) {
                
                // Begin crouching
                crouching = true;
                if (_mover.collider.GetType () == typeof (BoxCollider)) {

                    // Box collider
                    BoxCollider boxCollider = (BoxCollider)_mover.collider;
                    boxCollider.size = new Vector3 (boxCollider.size.x, _mover.moveData.defaultHeight * crouchingHeight, boxCollider.size.z);

                } else if (_mover.collider.GetType () == typeof (CapsuleCollider)) {

                    // Capsule collider
                    CapsuleCollider capsuleCollider = (CapsuleCollider)_mover.collider;
                    capsuleCollider.height = _mover.moveData.defaultHeight * crouchingHeight;

                }

                // Move position and stuff
                _mover.moveData.origin += heightDifference / 2 * (grounded ? Vector3.down : Vector3.up);
                foreach (Transform child in playerTransform) {

                    if (child == _mover.moveData.viewTransform)
                        continue;

                    child.localPosition = new Vector3 (child.localPosition.x, child.localPosition.y * crouchingHeight, child.localPosition.z);

                }

                uncrouchDown = !grounded;

            } else if (crouching) {

                // Check if the player can uncrouch
                bool canUncrouch = true;
                if (_mover.collider.GetType () == typeof (BoxCollider)) {

                    // Box collider
                    BoxCollider boxCollider = (BoxCollider)_mover.collider;
                    Vector3 halfExtents = boxCollider.size * 0.5f;
                    Vector3 startPos = boxCollider.transform.position;
                    Vector3 endPos = boxCollider.transform.position + (uncrouchDown ? Vector3.down : Vector3.up) * heightDifference;

                    Trace trace = Tracer.TraceBox (startPos, endPos, halfExtents, boxCollider.contactOffset, MovePhysics.groundLayerMask);

                    if (trace.hitCollider != null)
                        canUncrouch = false;

                } else if (_mover.collider.GetType () == typeof (CapsuleCollider)) {

                    // Capsule collider
                    CapsuleCollider capsuleCollider = (CapsuleCollider)_mover.collider;
                    Vector3 point1 = capsuleCollider.center + Vector3.up * capsuleCollider.height * 0.5f;
                    Vector3 point2 = capsuleCollider.center + Vector3.down * capsuleCollider.height * 0.5f;
                    Vector3 startPos = capsuleCollider.transform.position;
                    Vector3 endPos = capsuleCollider.transform.position + (uncrouchDown ? Vector3.down : Vector3.up) * heightDifference;

                    Trace trace = Tracer.TraceCapsule (point1, point2, capsuleCollider.radius, startPos, endPos, capsuleCollider.contactOffset, MovePhysics.groundLayerMask);

                    if (trace.hitCollider != null)
                        canUncrouch = false;

                }

                // Uncrouch
                if (canUncrouch && crouchLerp <= 0.9f) {

                    crouching = false;
                    if (_mover.collider.GetType () == typeof (BoxCollider)) {

                        // Box collider
                        BoxCollider boxCollider = (BoxCollider)_mover.collider;
                        boxCollider.size = new Vector3 (boxCollider.size.x, _mover.moveData.defaultHeight, boxCollider.size.z);

                    } else if (_mover.collider.GetType () == typeof (CapsuleCollider)) {

                        // Capsule collider
                        CapsuleCollider capsuleCollider = (CapsuleCollider)_mover.collider;
                        capsuleCollider.height = _mover.moveData.defaultHeight;

                    }

                    // Move position and stuff
                    _mover.moveData.origin += heightDifference / 2 * (uncrouchDown ? Vector3.down : Vector3.up);
                    foreach (Transform child in playerTransform) {

                        child.localPosition = new Vector3 (child.localPosition.x, child.localPosition.y / crouchingHeight, child.localPosition.z);

                    }

                }

                if (!canUncrouch)
                    crouchLerp = 1f;

            }

            // Changing camera position
            if (!crouching)
                _mover.moveData.viewTransform.localPosition = Vector3.Lerp (_mover.moveData.viewTransformDefaultLocalPos, _mover.moveData.viewTransformDefaultLocalPos * crouchingHeight + Vector3.down * heightDifference * 0.5f, crouchLerp);
            else
                _mover.moveData.viewTransform.localPosition = Vector3.Lerp (_mover.moveData.viewTransformDefaultLocalPos - Vector3.down * heightDifference * 0.5f, _mover.moveData.viewTransformDefaultLocalPos * crouchingHeight, crouchLerp);

        }

        void SlideMovement () {
            
            // Gradually change direction
            slideDirection += new Vector3 (groundNormal.x, 0f, groundNormal.z) * slideSpeedCurrent * _deltaTime;
            slideDirection = slideDirection.normalized;

            // Set direction
            Vector3 slideForward = Vector3.Cross (groundNormal, Quaternion.AngleAxis (-90, Vector3.up) * slideDirection);
            
            // Set the velocity
            slideSpeedCurrent -= _config.slideFriction * _deltaTime;
            slideSpeedCurrent = Mathf.Clamp (slideSpeedCurrent, 0f, _config.maximumSlideSpeed);
            slideSpeedCurrent -= (slideForward * slideSpeedCurrent).y * _deltaTime * _config.downhillSlideSpeedMultiplier; // Accelerate downhill (-y = downward, - * - = +)

            _mover.moveData.velocity = slideForward * slideSpeedCurrent;
            
            // Jump
            if (_mover.moveData.wishJump && slideSpeedCurrent < _config.minimumSlideSpeed * _config.slideSpeedMultiplier) {

                Jump ();
                return;

            }

        }

    }
}