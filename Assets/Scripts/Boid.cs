using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;

public class Boid : MonoBehaviour
{

    BoidSettings settings;

    // State
    [HideInInspector]
    public Vector3 position;
    [HideInInspector]
    public Vector3 forward;
    Vector3 velocity;

    public enum CheckState { NeedUpdate, Updated }
    public CheckState checkState =  CheckState.NeedUpdate;
    Vector3 accelerate;

    // To update:
    Vector3 acceleration;
    [HideInInspector]
    public Vector3 avgFlockHeading;
    [HideInInspector]
    public Vector3 avgAvoidanceHeading;
    [HideInInspector]
    public Vector3 centreOfFlockmates;
    [HideInInspector]
    public int numPerceivedFlockmates;

    // Cached
    Material material;
    Transform cachedTransform;
    Transform target;

    static int batchSize = 100;

    void Awake()
    {
        material = transform.GetComponentInChildren<MeshRenderer>().material;
        cachedTransform = transform;
    }

    public void Initialize(BoidSettings settings, Transform target)
    {
        this.target = target;
        this.settings = settings;

        position = cachedTransform.position;
        forward = cachedTransform.forward;

        float startSpeed = (settings.minSpeed + settings.maxSpeed) / 2;
        velocity = transform.forward * startSpeed;
    }

    public void SetColour(Color col)
    {
        if (material != null)
        {
            material.color = col;
        }
    }

    public void UpdateBoid()
    {
        Vector3 acceleration = Vector3.zero;

        if (target != null)
        {
            Vector3 offsetToTarget = (target.position - position);
            acceleration = SteerTowards(offsetToTarget) * settings.targetWeight;
        }

        if (numPerceivedFlockmates != 0)
        {
            centreOfFlockmates /= numPerceivedFlockmates;

            Vector3 offsetToFlockmatesCentre = (centreOfFlockmates - position);

            var alignmentForce = SteerTowards(avgFlockHeading) * settings.alignWeight;
            var cohesionForce = SteerTowards(offsetToFlockmatesCentre) * settings.cohesionWeight;
            var seperationForce = SteerTowards(avgAvoidanceHeading) * settings.seperateWeight;

            acceleration += alignmentForce;
            acceleration += cohesionForce;
            acceleration += seperationForce;
        }

        if (IsHeadingForCollision())
        {
            Vector3 collisionAvoidDir = ObstacleRaysJob();
            // Vector3 collisionAvoidDir = ObstacleRays ();
            Vector3 collisionAvoidForce = SteerTowards(collisionAvoidDir) * settings.avoidCollisionWeight;
            acceleration += collisionAvoidForce;
        }

        velocity += acceleration * Time.deltaTime;
        float speed = velocity.magnitude;
        Vector3 dir = velocity / speed;
        speed = Mathf.Clamp(speed, settings.minSpeed, settings.maxSpeed);
        velocity = dir * speed;

        cachedTransform.position += velocity * Time.deltaTime;
        cachedTransform.forward = dir;
        position = cachedTransform.position;
        forward = dir;
    }

    public static int FirstCheck(Boid[] boids){
        int count = boids.Length;
        var results = new NativeArray<RaycastHit>(count, Allocator.TempJob);
        var commands = new NativeArray<SpherecastCommand>(count, Allocator.TempJob);
        for(int i = 0; i < boids.Length; i++){
            commands[i] = new SpherecastCommand(
                boids[i].position,
                boids[i].settings.boundsRadius, 
                boids[i].forward, 
                boids[i].settings.collisionAvoidDst, 
                boids[i].settings.obstacleMask
            );
        }

        JobHandle handle = SpherecastCommand.ScheduleBatch(commands, results, batchSize, default(JobHandle));

        // Wait for the batch processing job to complete
        handle.Complete();

        // Copy the result. If batchedHit.collider is null there was no hit
        for (int i = 0; i < boids.Length; i++)
        {
            if (results[i].collider == null)
            {
                boids[i].checkState = CheckState.Updated;
                boids[i].accelerate = Vector3.zero;
                count--;
            } else {
                boids[i].checkState = CheckState.NeedUpdate;
            }
        }

        // Dispose the buffers
        results.Dispose();
        commands.Dispose();
        return count;
    }

    public static void avoidProcess(Boid[] boids)
    {
        int count = FirstCheck(boids);
        int index = 0;
        int[] map = new int[count];

        while(count > 0 && index < BoidHelper.directions.Length){

            Vector3 rayDirection = BoidHelper.directions[index];
            var results = new NativeArray<RaycastHit>(count, Allocator.TempJob);
            var commands = new NativeArray<SpherecastCommand>(count, Allocator.TempJob);

            int cmdId = 0;

            for (int i = 0; i < boids.Length; i++)
            {
                if(boids[i].checkState == CheckState.Updated){
                    continue;
                }
                Vector3 dir = boids[i].cachedTransform.TransformDirection(rayDirection);

                map[cmdId] = i;

                commands[cmdId] = new SpherecastCommand(
                    boids[i].position, 
                    boids[i].settings.boundsRadius, 
                    dir, 
                    boids[i].settings.collisionAvoidDst,
                    boids[i].settings.obstacleMask
                );
                cmdId++;
            }

            // Schedule the batch of raycasts
            JobHandle handle = SpherecastCommand.ScheduleBatch(commands, results, batchSize, default(JobHandle));

            // Wait for the batch processing job to complete
            handle.Complete();
            int currentCount = count;
            // Copy the result. If batchedHit.collider is null there was no hit
            for (int i = 0; i < currentCount; i++)
            {
                int mapId = map[i];
                if (results[i].collider == null)
                {
                    boids[mapId].checkState = CheckState.Updated;
                    boids[mapId].accelerate = boids[mapId].cachedTransform.TransformDirection(rayDirection);
                    count--;
                    break;
                }
            }

            // Dispose the buffers
            results.Dispose();
            commands.Dispose();
            index++;
        }
    }

    public static void updateBoids(Boid[] boids)
    {
        avoidProcess(boids);
        for (int i = 0; i < boids.Length; i++)
        {
            var boid = boids[i];

            Vector3 acceleration = Vector3.zero;


            if (boid.target != null)
            {
                Vector3 offsetToTarget = (boid.target.position - boid.position);
                acceleration = boid.SteerTowards(offsetToTarget) * boid.settings.targetWeight;
            }

            if (boid.numPerceivedFlockmates != 0)
            {
                boid.centreOfFlockmates /= boid.numPerceivedFlockmates;

                Vector3 offsetToFlockmatesCentre = (boid.centreOfFlockmates - boid.position);

                var alignmentForce = boid.SteerTowards(boid.avgFlockHeading) * boid.settings.alignWeight;
                var cohesionForce = boid.SteerTowards(offsetToFlockmatesCentre) * boid.settings.cohesionWeight;
                var seperationForce = boid.SteerTowards(boid.avgAvoidanceHeading) * boid.settings.seperateWeight;

                acceleration += alignmentForce;
                acceleration += cohesionForce;
                acceleration += seperationForce;
            }
            if(boid.accelerate.magnitude > 0){
                acceleration += boid.SteerTowards(boid.accelerate) * boid.settings.avoidCollisionWeight;
            }
            // Debug.Log(boid.accelerate);

            boid.velocity += acceleration * Time.deltaTime;
            float speed = boid.velocity.magnitude;
            Vector3 dir = boid.velocity / speed;
            speed = Mathf.Clamp(speed, boid.settings.minSpeed, boid.settings.maxSpeed);
            boid.velocity = dir * speed;

            boid.cachedTransform.position += boid.velocity * Time.deltaTime;
            boid.cachedTransform.forward = dir;
            boid.position = boid.cachedTransform.position;
            boid.forward = dir;
        }
    }

    bool IsHeadingForCollision()
    {
        RaycastHit hit;
        if (Physics.SphereCast(position, settings.boundsRadius, forward, out hit, settings.collisionAvoidDst, settings.obstacleMask))
        {
            return true;
        }
        else { }
        return false;
    }

    Vector3 ObstacleRays () {
        Vector3[] rayDirections = BoidHelper.directions;

        for (int i = 0; i < rayDirections.Length; i++) {
            Vector3 dir = cachedTransform.TransformDirection (rayDirections[i]);
            Ray ray = new Ray (position, dir);
            if (!Physics.SphereCast (ray, settings.boundsRadius, settings.collisionAvoidDst, settings.obstacleMask)) {
                return dir;
            }
        }

        return forward;
    }

    private Vector3 ObstacleRaysJob()
    {
        Vector3[] rayDirections = BoidHelper.directions;
        int count = rayDirections.Length;
        // count = 1;

        // Perform a single raycast using RaycastCommand and wait for it to complete
        // Setup the command and result buffers
        var results = new NativeArray<RaycastHit>(count, Allocator.TempJob);

        // var commands = new NativeArray<RaycastCommand>(count, Allocator.TempJob);
        var commands = new NativeArray<SpherecastCommand>(count, Allocator.TempJob);

        // Set the data of the first command
        Vector3 origin = position;
        for (int i = 0; i < count; i++)
        {
            Vector3 dir = cachedTransform.TransformDirection(rayDirections[i]);

            Vector3 direction = dir;

            commands[i] = new SpherecastCommand(origin, settings.boundsRadius, direction, settings.collisionAvoidDst, settings.obstacleMask);
        }

        // Schedule the batch of raycasts
        JobHandle handle = SpherecastCommand.ScheduleBatch(commands, results, batchSize, default(JobHandle));

        // Wait for the batch processing job to complete
        handle.Complete();
        Vector3 result = forward;
        // Copy the result. If batchedHit.collider is null there was no hit
        for (int i = 0; i < count; i++)
        {
            if (results[i].collider == null)
            {
                result = cachedTransform.TransformDirection(rayDirections[i]);
                break;
            }
        }

        // Dispose the buffers
        results.Dispose();
        commands.Dispose();
        return result;
    }

    Vector3 SteerTowards(Vector3 vector)
    {
        Vector3 v = vector.normalized * settings.maxSpeed - velocity;
        return Vector3.ClampMagnitude (v, settings.maxSteerForce);
    }

}