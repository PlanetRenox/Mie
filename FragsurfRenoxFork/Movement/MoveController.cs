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

        // Static vectors to reduce allocations in hot paths
        private static readonly Vector3 vectorUp = Vector3.up;
        private static readonly Vector3 vectorDown = Vector3.down;
        private static readonly Vector3 horizontalPlane = new Vector3(1f, 0f, 1f);

        ///// Methods /////

        Vector3 groundNormal = Vector3.up;

        /// <summary>
        /// Process movement for the controllable entity
        /// </summary>
        /// <param name="mover">Entity to move</param>
        /// <param name="config">Movement configuration</param>
        /// <param name="deltaTime">Time since last frame</param>
        public void ProcessMovement(IMoveControllable mover, MovementConfig config, float deltaTime) {
            // Early exit if any parameter is null
            if (mover == null || config == null || deltaTime <= 0) {
                return;
            }
            
            // Cache parameters to avoid repeated passing
            _mover = mover;
            _config = config;
            _deltaTime = deltaTime;
            
            // Handle ladder detection if enabled
            if (_mover.moveData.laddersEnabled) {
                ProcessLadders();
            }
            
            // Choose movement processing based on environment
            if (_mover.moveData.underwater) {
                UnderwaterPhysics();
            }
            else if (!_mover.moveData.climbingLadder) {
                // Regular movement processing
                ProcessRegularMovement();
            }

            // Apply velocity clamping and collisions
            ProcessVelocityAndCollisions();

            // Save previous grounded state
            _mover.moveData.groundedTemp = _mover.moveData.grounded;

            // Clear mover reference to avoid holding references
            _mover = null;
        }

        /// <summary>
        /// Process ladder detection and movement
        /// </summary>
        private void ProcessLadders() {
            if (!_mover.moveData.climbingLadder) {
                // Look for ladders when not already climbing
                Vector3 scaledVelocity = _mover.moveData.velocity * Mathf.Clamp(_deltaTime * 2f, 0.025f, 0.25f);
                LadderCheck(new Vector3(1f, 0.95f, 1f), scaledVelocity);
            } else {
                // Already on ladder, process ladder physics
                LadderPhysics();
            }
        }

        /// <summary>
        /// Process regular (non-underwater, non-ladder) movement
        /// </summary>
        private void ProcessRegularMovement() {
            // Reset jumping state when falling
            if (_mover.moveData.velocity.y <= 0f) {
                jumping = false;
            }

            // Apply gravity when not grounded
            if (_mover.groundObject == null) {
                float gravity = _config.gravity * _deltaTime * _mover.moveData.gravityFactor;
                _mover.moveData.velocity.y -= gravity;
                _mover.moveData.velocity.y += _mover.baseVelocity.y * _deltaTime;
            }
            
            // Update ground state and calculate movement
            CheckGrounded();
            CalculateMovementVelocity();
        }

        /// <summary>
        /// Process velocity clamping and collision handling
        /// </summary>
        private void ProcessVelocityAndCollisions() {
            // Cache the y component of velocity for restoration after horizontal clamping
            float yVel = _mover.moveData.velocity.y;
            
            // Temporarily zero out y for horizontal magnitude calculation
            _mover.moveData.velocity.y = 0f;
            
            // Clamp horizontal velocity magnitude
            _mover.moveData.velocity = Vector3.ClampMagnitude(_mover.moveData.velocity, _config.maxVelocity);
            
            // Store overall speed for external use
            speed = _mover.moveData.velocity.magnitude;
            
            // Restore the y component
            _mover.moveData.velocity.y = yVel;
            
            // Handle collisions based on whether the player is moving
            if (_mover.moveData.velocity.sqrMagnitude < 0.0001f) {
                // Handle collisions when stationary
                MovePhysics.ResolveCollisions(
                    _mover.collider, 
                    ref _mover.moveData.origin, 
                    ref _mover.moveData.velocity, 
                    _mover.moveData.rigidbodyPushForce, 
                    1f, 
                    _mover.moveData.stepOffset, 
                    _mover
                );
            } else {
                // Process movement in small steps to avoid tunneling
                ProcessMovementWithCollisions();
            }
        }

        /// <summary>
        /// Process movement in small steps to handle collisions
        /// </summary>
        private void ProcessMovementWithCollisions() {
            const float maxDistPerFrame = 0.2f;
            
            // Calculate velocity for this frame
            Vector3 velocityThisFrame = _mover.moveData.velocity * _deltaTime;
            float velocityDistLeft = velocityThisFrame.magnitude;
            
            // Early exit if no movement
            if (velocityDistLeft < 0.0001f) {
                return;
            }
            
            float initialVel = velocityDistLeft;
            
            // Process movement in small chunks
            while (velocityDistLeft > 0f) {
                // Calculate amount to move in this iteration
                float amountThisLoop = Mathf.Min(maxDistPerFrame, velocityDistLeft);
                velocityDistLeft -= amountThisLoop;

                // Calculate the proportion of velocity to apply
                float ratio = amountThisLoop / initialVel;
                
                // Apply movement for this step
                Vector3 velThisLoop = velocityThisFrame * ratio;
                _mover.moveData.origin += velThisLoop;

                // Handle collisions for this movement step
                MovePhysics.ResolveCollisions(
                    _mover.collider, 
                    ref _mover.moveData.origin, 
                    ref _mover.moveData.velocity, 
                    _mover.moveData.rigidbodyPushForce, 
                    ratio, 
                    _mover.moveData.stepOffset, 
                    _mover
                );
            }
        }

        /// <summary>
        /// Calculate movement velocity based on input and environment
        /// </summary>
        private void CalculateMovementVelocity() {
            switch (_mover.moveType) {
                case MoveType.Walk:
                    if (_mover.groundObject == null) {
                        // AIR MOVEMENT
                        ProcessAirMovement();
                    } else {
                        // GROUND MOVEMENT
                        ProcessGroundMovement();
                    }
                    break;
            }
        }

        /// <summary>
        /// Process movement while in the air
        /// </summary>
        private void ProcessAirMovement() {
            // Reset sliding state
            wasSliding = false;

            // Apply air movement from input
            _mover.moveData.velocity += AirInputMovement();

            // Handle collision reflection
            MovePhysics.Reflect(ref _mover.moveData.velocity, _mover.collider, _mover.moveData.origin, _deltaTime);
        }

        /// <summary>
        /// Process movement while on the ground
        /// </summary>
        private void ProcessGroundMovement() {
            // Check for sliding
            if (ShouldStartSliding()) {
                SlideMovement();
                return;
            }

            // Update slide delay if needed
            UpdateSlideDelay();

            // Calculate friction and acceleration based on crouch state
            float friction = crouching ? _config.crouchFriction : _config.friction;
            float acceleration = crouching ? _config.crouchAcceleration : _config.deceleration;
            
            // Get movement directions
            Vector3 forward = Vector3.Cross(groundNormal, -playerTransform.right);
            Vector3 right = Vector3.Cross(groundNormal, forward);

            // Calculate speed based on movement state
            float moveSpeed = CalculateMoveSpeed();

            // Check for jumping
            if (_mover.moveData.wishJump) {
                ApplyFriction(0.0f, true, true);
                Jump();
                return;
            } else {
                ApplyFriction(1.0f * frictionMult, true, true);
            }

            // Calculate wish direction from input
            Vector3 wishDir = CalculateWishDirection(forward, right);
            
            // Set the target speed
            float wishSpeed = wishDir.magnitude * moveSpeed;

            // Store vertical velocity for restoring after horizontal acceleration
            float yVel = _mover.moveData.velocity.y;
            
            // Apply acceleration in the desired direction
            Accelerate(wishDir, wishSpeed, acceleration * Mathf.Min(frictionMult, 1f), false);

            // Clamp horizontal velocity
            _mover.moveData.velocity = Vector3.ClampMagnitude(
                new Vector3(_mover.moveData.velocity.x, 0f, _mover.moveData.velocity.z), 
                _config.maxVelocity
            );
            
            // Restore vertical velocity
            _mover.moveData.velocity.y = yVel;

            // Apply slope effects
            ApplySlopeEffects(wishDir);
        }
        
        /// <summary>
        /// Check if player should start sliding
        /// </summary>
        private bool ShouldStartSliding() {
            // Update slide direction if not already sliding
            if (!wasSliding) {
                Vector3 horizontalVelocity = Vector3.Scale(_mover.moveData.velocity, horizontalPlane);
                slideDirection = horizontalVelocity.normalized;
                slideSpeedCurrent = Mathf.Max(_config.maximumSlideSpeed, horizontalVelocity.magnitude);
            }

            // Check if conditions for sliding are met
            sliding = false;
            if (_mover.moveData.velocity.magnitude > _config.minimumSlideSpeed && 
                _mover.moveData.slidingEnabled && 
                _mover.moveData.crouching && 
                slideDelay <= 0f) {
                
                // Initialize slide speed if starting a new slide
                if (!wasSliding) {
                    slideSpeedCurrent = Mathf.Clamp(
                        slideSpeedCurrent * _config.slideSpeedMultiplier, 
                        _config.minimumSlideSpeed, 
                        _config.maximumSlideSpeed
                    );
                }

                sliding = true;
                wasSliding = true;
                return true;
            }
            
            return false;
        }
        
        /// <summary>
        /// Update slide delay timer
        /// </summary>
        private void UpdateSlideDelay() {
            if (slideDelay > 0f) {
                slideDelay -= _deltaTime;
            }
            
            if (wasSliding) {
                slideDelay = _config.slideDelay;
                wasSliding = false;
            }
        }
        
        /// <summary>
        /// Calculate movement speed based on player state
        /// </summary>
        private float CalculateMoveSpeed() {
            if (_mover.moveData.sprinting) {
                return _config.sprintSpeed;
            } else if (crouching) {
                return _config.crouchSpeed;
            } else {
                return _config.walkSpeed;
            }
        }
        
        /// <summary>
        /// Calculate wish direction from input
        /// </summary>
        private Vector3 CalculateWishDirection(Vector3 forward, Vector3 right) {
            float forwardMove = _mover.moveData.verticalAxis;
            float rightMove = _mover.moveData.horizontalAxis;

            Vector3 wishDir = (forwardMove * forward + rightMove * right).normalized;
            return wishDir;
        }
        
        /// <summary>
        /// Apply effects of moving on slopes
        /// </summary>
        private void ApplySlopeEffects(Vector3 wishDir) {
            // Calculate forward velocity on the slope
            Vector3 horizontalVelocity = new Vector3(_mover.moveData.velocity.x, 0f, _mover.moveData.velocity.z);
            Vector3 forwardVelocity = Vector3.Cross(
                groundNormal, 
                Quaternion.AngleAxis(-90, Vector3.up) * horizontalVelocity
            );
            
            // Calculate how slopes should affect Y velocity
            float yVelocityNew = forwardVelocity.normalized.y * horizontalVelocity.magnitude;

            // Apply slope Y-velocity with a boost when moving downhill
            float downhillFactor = wishDir.y < 0f ? 1.2f : 1.0f;
            _mover.moveData.velocity.y = yVelocityNew * downhillFactor;
        }

        /// <summary>
        /// Handle underwater physics and movement
        /// </summary>
        private void UnderwaterPhysics() {
            // Apply velocity damping when underwater
            _mover.moveData.velocity = Vector3.Lerp(
                _mover.moveData.velocity, 
                Vector3.zero, 
                _config.underwaterVelocityDampening * _deltaTime
            );

            // Apply underwater gravity when not grounded
            if (!CheckGrounded()) {
                _mover.moveData.velocity.y -= _config.underwaterGravity * _deltaTime;
            }

            // Allow swimming upwards when jump button is pressed
            if (Input.GetButton("Jump")) {
                _mover.moveData.velocity.y += _config.swimUpSpeed * _deltaTime;
            }

            // Apply underwater friction
            ApplyFriction(1f, true, false);

            // Calculate movement directions
            Vector3 forward = Vector3.Cross(groundNormal, -playerTransform.right);
            Vector3 right = Vector3.Cross(groundNormal, forward);

            // Calculate wish direction from input
            float forwardMove = _mover.moveData.verticalAxis;
            float rightMove = _mover.moveData.horizontalAxis;
            
            Vector3 wishDir = (forwardMove * forward + rightMove * right).normalized;
            
            // Calculate forward velocity on slope
            Vector3 horizontalVelocity = new Vector3(_mover.moveData.velocity.x, 0f, _mover.moveData.velocity.z);
            Vector3 forwardVelocity = Vector3.Cross(
                groundNormal, 
                Quaternion.AngleAxis(-90, Vector3.up) * horizontalVelocity
            );

            // Set target speed and accelerate
            float wishSpeed = wishDir.magnitude * _config.underwaterSwimSpeed;

            // Cache y velocity for restoration after horizontal acceleration
            float yVel = _mover.moveData.velocity.y;
            
            // Apply acceleration
            Accelerate(wishDir, wishSpeed, _config.underwaterAcceleration, false);

            // Clamp horizontal velocity
            _mover.moveData.velocity = Vector3.ClampMagnitude(horizontalPlane, _config.maxVelocity);
            
            // Restore vertical velocity
            _mover.moveData.velocity.y = yVel;

            // Calculate and apply slope effects
            ApplyUnderwaterSlopeEffects(forwardVelocity, yVel);

            // Handle jumping out of water
            CheckWaterJump();
        }
        
        /// <summary>
        /// Apply slope effects when underwater
        /// </summary>
        private void ApplyUnderwaterSlopeEffects(Vector3 forwardVelocity, float yVel) {
            // Calculate horizontal velocity
            Vector3 horizontalVelocity = new Vector3(_mover.moveData.velocity.x, 0f, _mover.moveData.velocity.z);
            
            // Calculate slope Y contribution
            float slopeYVelocity = 0;
            if (forwardVelocity.normalized.sqrMagnitude > 0.0001f) {
                slopeYVelocity = forwardVelocity.normalized.y * horizontalVelocity.magnitude;
            }
            
            // Apply Y-velocity with slope contribution, but only add positive slope effects
            _mover.moveData.velocity.y = Mathf.Min(
                Mathf.Max(0f, slopeYVelocity) + yVel, 
                _config.underwaterSwimSpeed
            );
        }
        
        /// <summary>
        /// Check if player can jump out of water
        /// </summary>
        private void CheckWaterJump() {
            // Only allow jumping when moving forward and jump is pressed
            bool movingForwards = playerTransform.InverseTransformVector(_mover.moveData.velocity).z > 0f;
            if (!movingForwards || !Input.GetButton("Jump") || _mover.moveData.cameraUnderwater) {
                return;
            }
            
            // Check for surface ahead to jump onto
            Trace waterJumpTrace = TraceBounds(
                playerTransform.position, 
                playerTransform.position + playerTransform.forward * 0.1f, 
                MovePhysics.groundLayerMask
            );
            
            // Apply jump force if hitting a steep enough surface
            if (waterJumpTrace.hitCollider != null && 
                Vector3.Angle(Vector3.up, waterJumpTrace.planeNormal) >= _config.slopeLimit) {
                _mover.moveData.velocity.y = Mathf.Max(
                    _mover.moveData.velocity.y, 
                    _config.jumpForce
                );
            }
        }
        
        /// <summary>
        /// Check for ladders in the movement path
        /// </summary>
        /// <param name="colliderScale">Scale factor for collision checks</param>
        /// <param name="direction">Direction to check</param>
        private void LadderCheck(Vector3 colliderScale, Vector3 direction) {
            // Early exit if not moving
            if (direction.sqrMagnitude <= 0.0001f)
                return;
            
            // Scale horizontal direction only
            Vector3 horizontalDirection = Vector3.Scale(direction, horizontalPlane);
            bool foundLadder = false;

            // Cast a box to find potential ladders
            RaycastHit[] hits = Physics.BoxCastAll(
                _mover.moveData.origin,
                Vector3.Scale(_mover.collider.bounds.size * 0.5f, colliderScale),
                horizontalDirection.normalized,
                Quaternion.identity,
                horizontalDirection.magnitude,
                MovePhysics.groundLayerMask,
                QueryTriggerInteraction.Collide
            );

            // Check each hit for a ladder
            foreach (RaycastHit hit in hits) {
                Ladder ladder = hit.transform.GetComponentInParent<Ladder>();
                if (ladder != null) {
                    // Check if ladder is valid for climbing
                    if (IsValidLadder(hit)) {
                        foundLadder = true;
                        
                        // Handle new ladder detection
                        if (!_mover.moveData.climbingLadder) {
                            InitializeLadderClimb(hit, direction);
                        }
                    }
                }
            }

            // Reset ladder state if no valid ladder found
            if (!foundLadder) {
                ResetLadderState();
            }
        }

        /// <summary>
        /// Check if ladder is valid for climbing based on angle
        /// </summary>
        /// <param name="hit">Raycast hit information</param>
        /// <returns>True if ladder can be climbed</returns>
        private bool IsValidLadder(RaycastHit hit) {
            bool allowClimb = true;
            float ladderAngle = Vector3.Angle(Vector3.up, hit.normal);
            
            if (_mover.moveData.angledLaddersEnabled) {
                // For angled ladders, normal must point upward
                if (hit.normal.y < 0f) {
                    allowClimb = false;
                } else if (ladderAngle <= _mover.moveData.slopeLimit) {
                    // If angle is less than slope limit, treat as walkable surface
                    allowClimb = false;
                }
            } else if (hit.normal.y != 0f) {
                // For non-angled ladders, normal must be perfectly vertical
                allowClimb = false;
            }
            
            return allowClimb;
        }

        /// <summary>
        /// Initialize ladder climbing state
        /// </summary>
        /// <param name="hit">Raycast hit information</param>
        /// <param name="direction">Movement direction</param>
        private void InitializeLadderClimb(RaycastHit hit, Vector3 direction) {
            _mover.moveData.climbingLadder = true;
            _mover.moveData.ladderNormal = hit.normal;
            _mover.moveData.ladderDirection = -hit.normal * direction.magnitude * 2f;

            if (_mover.moveData.angledLaddersEnabled) {
                // Calculate climb direction for angled ladders
                Vector3 sideDir = hit.normal;
                sideDir.y = 0f;
                sideDir = Quaternion.AngleAxis(-90f, Vector3.up) * sideDir;

                _mover.moveData.ladderClimbDir = Quaternion.AngleAxis(90f, sideDir) * hit.normal;
                
                // Make sure Y is always 1 for consistent climb speed
                if (Mathf.Abs(_mover.moveData.ladderClimbDir.y) > 0.0001f) {
                    _mover.moveData.ladderClimbDir /= _mover.moveData.ladderClimbDir.y;
                }
            } else {
                // Vertical ladders use standard up direction
                _mover.moveData.ladderClimbDir = Vector3.up;
            }
        }

        /// <summary>
        /// Reset ladder state when no ladder found
        /// </summary>
        private void ResetLadderState() {
            _mover.moveData.ladderNormal = Vector3.zero;
            _mover.moveData.ladderVelocity = Vector3.zero;
            _mover.moveData.climbingLadder = false;
            _mover.moveData.ladderClimbDir = Vector3.up;
        }

        /// <summary>
        /// Handle physics while on a ladder
        /// </summary>
        private void LadderPhysics() {
            // Calculate climbing velocity based on input
            _mover.moveData.ladderVelocity = _mover.moveData.ladderClimbDir * _mover.moveData.verticalAxis * 6f;

            // Smoothly transition to ladder velocity
            _mover.moveData.velocity = Vector3.Lerp(
                _mover.moveData.velocity, 
                _mover.moveData.ladderVelocity, 
                _deltaTime * 10f
            );

            // Continue checking for ladder collision
            LadderCheck(Vector3.one, _mover.moveData.ladderDirection);
            
            // Check for ground when moving down
            Trace floorTrace = TraceToFloor();
            if (_mover.moveData.verticalAxis < 0f && 
                floorTrace.hitCollider != null && 
                Vector3.Angle(Vector3.up, floorTrace.planeNormal) <= _mover.moveData.slopeLimit) {
                // Exit ladder when moving down and reaching walkable ground
                _mover.moveData.velocity = _mover.moveData.ladderNormal * 0.5f;
                _mover.moveData.ladderVelocity = Vector3.zero;
                _mover.moveData.climbingLadder = false;
            }

            // Handle jumping off ladder
            if (_mover.moveData.wishJump) {
                _mover.moveData.velocity = _mover.moveData.ladderNormal * 4f;
                _mover.moveData.ladderVelocity = Vector3.zero;
                _mover.moveData.climbingLadder = false;
            }
        }
        
        /// <summary>
        /// Accelerate movement in the desired direction
        /// </summary>
        /// <param name="wishDir">Desired movement direction</param>
        /// <param name="wishSpeed">Desired movement speed</param>
        /// <param name="acceleration">Acceleration value</param>
        /// <param name="yMovement">Whether to apply movement to the Y axis</param>
        private void Accelerate(Vector3 wishDir, float wishSpeed, float acceleration, bool yMovement) {
            // Skip if no wishdir
            if (wishDir.sqrMagnitude < 0.0001f) {
                return;
            }

            // Calculate dot product to determine current speed in the desired direction
            float currentSpeed = Vector3.Dot(_mover.moveData.velocity, wishDir);
            
            // Calculate remaining speed to reach target
            float addSpeed = wishSpeed - currentSpeed;

            // If no speed to add, return
            if (addSpeed <= 0) {
                return;
            }

            // Calculate acceleration amount
            float accelSpeed = Mathf.Min(acceleration * _deltaTime * wishSpeed, addSpeed);
            
            // Apply acceleration to velocity
            _mover.moveData.velocity.x += accelSpeed * wishDir.x;
            if (yMovement) { 
                _mover.moveData.velocity.y += accelSpeed * wishDir.y; 
            }
            _mover.moveData.velocity.z += accelSpeed * wishDir.z;
        }

        /// <summary>
        /// Apply friction to velocity
        /// </summary>
        /// <param name="t">Friction multiplier</param>
        /// <param name="yAffected">Whether Y component should be affected</param>
        /// <param name="grounded">Whether the entity is on the ground</param>
        private void ApplyFriction(float t, bool yAffected, bool grounded) {
            // Extract horizontal velocity
            Vector3 vel = _mover.moveData.velocity;
            vel.y = 0.0f;
            
            float speed = vel.magnitude;
            if (speed < 0.0001f) {
                return; // No need to apply friction if not moving
            }
            
            float drop = 0.0f;

            // Get friction and deceleration values based on crouch state
            float friction = crouching ? _config.crouchFriction : _config.friction;
            float decel = crouching ? _config.crouchDeceleration : _config.deceleration;

            // Apply friction calculation only when grounded
            if (grounded) {
                // Calculate control based on speed vs deceleration
                float control = speed < decel ? decel : speed;
                drop = control * friction * _deltaTime * t;
            }

            // Calculate new speed after friction
            float newSpeed = Mathf.Max(speed - drop, 0f);
            
            // Apply the friction factor to velocity
            if (speed > 0.0001f) {
                float frictionFactor = newSpeed / speed;
                // Apply to each component
                _mover.moveData.velocity.x *= frictionFactor;
                if (yAffected) { 
                    _mover.moveData.velocity.y *= frictionFactor; 
                }
                _mover.moveData.velocity.z *= frictionFactor;
            }
        }

        /// <summary>
        /// Apply movement input while in the air
        /// </summary>
        /// <returns>Additional velocity from air movement</returns>
        private Vector3 AirInputMovement() {
            Vector3 wishVel, wishDir;
            float wishSpeed;

            // Get movement intent
            GetWishValues(out wishVel, out wishDir, out wishSpeed);
            
            // Air speed clamping
            if (_config.clampAirSpeed && wishSpeed > 0.0001f && wishSpeed > _config.maxSpeed) {
                wishVel *= (_config.maxSpeed / wishSpeed);
                wishSpeed = _config.maxSpeed;
            }

            // Apply air acceleration
            return MovePhysics.AirAccelerate(
                _mover.moveData.velocity, 
                wishDir, 
                wishSpeed, 
                _config.airAcceleration, 
                _config.airCap, 
                _deltaTime
            );
        }

        /// <summary>
        /// Calculate desired movement values from input
        /// </summary>
        /// <param name="wishVel">Output desired velocity</param>
        /// <param name="wishDir">Output desired direction</param>
        /// <param name="wishSpeed">Output desired speed</param>
        private void GetWishValues(out Vector3 wishVel, out Vector3 wishDir, out float wishSpeed) {
            // Initialize outputs
            wishVel = Vector3.zero;
            wishDir = Vector3.zero;
            wishSpeed = 0f;

            // Get movement vectors
            Vector3 forward = _mover.forward;
            Vector3 right = _mover.right;

            // Flatten vectors for horizontal movement
            forward.y = 0;
            right.y = 0;
            
            // Normalize if they have magnitude
            if (forward.sqrMagnitude > 0.0001f) {
                forward.Normalize();
            }
            if (right.sqrMagnitude > 0.0001f) {
                right.Normalize();
            }

            // Calculate velocity based on input
            wishVel = forward * _mover.moveData.forwardMove + right * _mover.moveData.sideMove;
            wishVel.y = 0;

            // Calculate speed and direction
            wishSpeed = wishVel.magnitude;
            if (wishSpeed > 0.0001f) {
                wishDir = wishVel / wishSpeed;
            } else {
                wishDir = Vector3.zero;
            }
        }

        /// <summary>
        /// Apply jump force to the entity's velocity
        /// </summary>
        private void Jump() {
            // Reset wish jump flag if auto bunnyhopping is disabled
            if (!_config.autoBhop) {
                _mover.moveData.wishJump = false;
            }
            
            // Apply jump force to Y velocity
            _mover.moveData.velocity.y += _config.jumpForce;
            jumping = true;
        }

        /// <summary>
        /// Check if the entity is on the ground
        /// </summary>
        /// <returns>True if the entity is grounded</returns>
        private bool CheckGrounded() {
            // Set default surface friction
            _mover.moveData.surfaceFriction = 1f;
            
            // Cache upward movement check
            bool movingUp = _mover.moveData.velocity.y > 0f;
            
            // Cast ray to check for ground below
            Trace trace = TraceToFloor();
            float groundSteepness = Vector3.Angle(vectorUp, trace.planeNormal);

            // Check if not grounded
            if (trace.hitCollider == null || 
                groundSteepness > _config.slopeLimit || 
                (jumping && movingUp)) {
                
                // Set to not grounded
                SetGround(null);

                // Apply air friction when moving upward in air
                if (movingUp && _mover.moveType != MoveType.Noclip) {
                    _mover.moveData.surfaceFriction = _config.airFriction;
                }
                
                return false;
            } else {
                // Set ground normal and apply ground
                groundNormal = trace.planeNormal;
                SetGround(trace.hitCollider.gameObject);
                return true;
            }
        }

        /// <summary>
        /// Set the ground object for the entity
        /// </summary>
        /// <param name="obj">GameObject to set as ground, or null</param>
        private void SetGround(GameObject obj) {
            if (obj != null) {
                // Set ground object and zero vertical velocity
                _mover.groundObject = obj;
                _mover.moveData.velocity.y = 0;
                _mover.moveData.grounded = true;
            } else {
                // Clear ground object
                _mover.groundObject = null;
                _mover.moveData.grounded = false;
            }
        }

        /// <summary>
        /// Trace a short ray downward to check for ground
        /// </summary>
        /// <returns>Trace result for ground check</returns>
        private Trace TraceToFloor() {
            // Create position slightly below current origin
            Vector3 down = _mover.moveData.origin;
            down.y -= 0.15f;

            // Cast ray down to detect ground
            return Tracer.TraceCollider(_mover.collider, _mover.moveData.origin, down, MovePhysics.groundLayerMask);
        }

        /// <summary>
        /// Trace between two points with the entity's collider
        /// </summary>
        /// <param name="start">Starting position</param>
        /// <param name="end">End position</param>
        /// <param name="layerMask">Layer mask for collision check</param>
        /// <returns>Trace result</returns>
        private Trace TraceBounds(Vector3 start, Vector3 end, int layerMask) {
            return Tracer.TraceCollider(_mover.collider, start, end, layerMask);
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

        /// <summary>
        /// Handle movement while sliding
        /// </summary>
        private void SlideMovement() {
            // Get horizontal components of ground normal
            Vector3 horizontalNormal = new Vector3(groundNormal.x, 0f, groundNormal.z);
            
            // Gradually change slide direction based on ground slope
            if (horizontalNormal.sqrMagnitude > 0.0001f) {
                slideDirection += horizontalNormal * slideSpeedCurrent * _deltaTime;
                slideDirection = slideDirection.normalized;
            }

            // Calculate sliding forward direction using cross product
            Vector3 slideForward = Vector3.Cross(
                groundNormal, 
                Quaternion.AngleAxis(-90, vectorUp) * slideDirection
            );
            
            // Apply slide friction
            slideSpeedCurrent -= _config.slideFriction * _deltaTime;
            
            // Clamp slide speed
            slideSpeedCurrent = Mathf.Clamp(slideSpeedCurrent, 0f, _config.maximumSlideSpeed);
            
            // Accelerate when sliding downhill
            if (slideForward.y < 0) {
                float downhillAcceleration = -slideForward.y * slideSpeedCurrent * 
                                            _deltaTime * _config.downhillSlideSpeedMultiplier;
                slideSpeedCurrent += downhillAcceleration;
            }

            // Set velocity based on slide direction and speed
            _mover.moveData.velocity = slideForward * slideSpeedCurrent;
            
            // Allow jumping out of slide
            if (_mover.moveData.wishJump && 
                slideSpeedCurrent < _config.minimumSlideSpeed * _config.slideSpeedMultiplier) {
                Jump();
            }
        }

    }
}