using System.Collections;
using System.Threading;
using System.Collections.Generic;
using UnityEngine;

abstract class Constraint
{
    public Constraint()
    {

    }

    public abstract void EnforceConstraint();
}

class Point
{
    public Vector3 position;

    public Point(Vector3 position)
    {
        this.position = position;
    }

    public Point() :
        this(new Vector3(0.0f, 0.0f, 0.0f))
    {

    }
}

class DistanceConstraint : Constraint
{

    private Point point1;
    private Point point2;
    private readonly float maxDistance;
    private readonly float maxDistanceSquared;
    private readonly float minDistance;
    private readonly float minDistanceSquared;

    public DistanceConstraint(Point point1, Point point2, float minDistance, float maxDistance) :
        base()
    {
        this.point1 = point1;
        this.point2 = point2;
        this.maxDistance = maxDistance;
        this.minDistance = minDistance;
        maxDistanceSquared = maxDistance * maxDistance;
        minDistanceSquared = minDistance * minDistance;
    }

    public override void EnforceConstraint()
    {
        float distance = Vector3.Distance(point1.position, point2.position);

        
        if (distance < minDistance)
        {
            Vector3 delta = point2.position - point1.position;
            float deltaLength = Mathf.Sqrt(Vector3.Dot(delta, delta));
            float diff = (deltaLength - minDistance) / deltaLength;
            point1.position += delta * 0.5f * diff;
            point2.position -= delta * 0.5f * diff;
        } else if (distance > maxDistance)
        {

            Vector3 delta = point2.position - point1.position;
            float deltaLength = Mathf.Sqrt(Vector3.Dot(delta, delta));
            float diff = (deltaLength - maxDistance) / deltaLength;
            point1.position += delta * 0.5f * diff;
            point2.position -= delta * 0.5f * diff;
        }
    }
}

class BoxConstraint : Constraint
{
    private Bounds bounds;
    private Point point;

    public BoxConstraint(Point point, Bounds bounds) :
        base()
    {
        this.bounds = bounds;
        this.point = point;
    }

    public override void EnforceConstraint()
    {
        Vector3 newVector = Vector3.Min(Vector3.Max(point.position, bounds.min), bounds.max);

        point.position.x = newVector.x;
        point.position.y = newVector.y;
        point.position.z = newVector.z;

    }
}

class AnchorConstraint : Constraint
{
    private Point point;
    private Vector3 anchorPosition;
    private Vector3 maxDistance;

    public AnchorConstraint(Point point, Vector3 maxDistance)
    {
        this.point = point;
        this.anchorPosition = point.position;
        this.maxDistance = maxDistance;
    }

    public override void EnforceConstraint()
    {
        Vector3 newVector = Vector3.Min(Vector3.Max(point.position, anchorPosition - (maxDistance * 0.5f)), anchorPosition + (maxDistance * 0.5f));

        point.position.x = newVector.x;
        point.position.y = newVector.y;
        point.position.z = newVector.z;
    }
}

public class DeformableMesh : MonoBehaviour
{
    public int numIterations = 10;
    public bool bumpMapCompatible = true;
    public float drift = 0.0f;
    public float brakeDistance = 0.01f;
    private Point[] points;
    private Point[] pointsPrevious;
    private Vector3[] forceAccumulator;
    private Constraint[] constraints;
    private Mesh mesh;
    private Vector2[] uvs;
    private int[] triangles;
    private Vector3[] thisFrameForce;

    // Start is called before the first frame update
    void Start()
    {
        mesh = GetComponent<MeshFilter>().mesh;

        if (!mesh)
            return;

        mesh.MarkDynamic();

        Vector3[] vertices = mesh.vertices;
        uvs = mesh.uv;
        triangles = mesh.triangles;

        points = new Point[vertices.Length];
        pointsPrevious = new Point[vertices.Length];
        forceAccumulator = new Vector3[vertices.Length];
        thisFrameForce = new Vector3[vertices.Length];

        constraints = new Constraint[vertices.Length * 2];
        Bounds bounds = mesh.bounds;
        Vector3 anchorDistance = (bounds.max - bounds.min) * 0.5f * drift;
        for (int i = 0; i < vertices.Length; i++)
        {
            points[i] = new Point(vertices[i]);
            pointsPrevious[i] = new Point(vertices[i]);
            forceAccumulator[i] = new Vector3();
            thisFrameForce[i] = new Vector3();
            constraints[i * 2] = new BoxConstraint(points[i], bounds);
            constraints[i * 2 + 1] = new AnchorConstraint(points[i], anchorDistance);
        }
    }


    private void Verlet()
    {
        for (int i = 0; i < points.Length; i++)
        {
            if ((points[i].position - pointsPrevious[i].position).magnitude < brakeDistance)
            {
                pointsPrevious[i].position = points[i].position;
            }

            Vector3 temp = new Vector3(points[i].position.x, points[i].position.y, points[i].position.z);
            points[i].position += points[i].position - pointsPrevious[i].position + thisFrameForce[i] * Time.deltaTime * Time.deltaTime;
            pointsPrevious[i].position = temp;
        }
    }

    //Doesn't actually do anything besides set the thisFrameForce and flush the force accumulator, could be used to accumulate forces for any mesh-wide physics effects
    private void AccumulateForces()
    {
        for (int i=0; i < points.Length; i++)
        {
            thisFrameForce[i] = forceAccumulator[i];
            forceAccumulator[i].x = 0.0f;
            forceAccumulator[i].y = 0.0f;
            forceAccumulator[i].z = 0.0f;
        }
    }

    private void SatisfyConstraints()
    {
        for (int i = 0; i < numIterations; i++)
        {
            foreach (Constraint constraint in constraints)
            {
                constraint.EnforceConstraint();
            }
        }
    }

    private void UpdateMesh()
    {
        Vector3[] newVertices = new Vector3[points.Length];
        for (int i = 0; i < points.Length; i++)
        {
            newVertices[i] = points[i].position;
        }

        mesh.Clear();
        mesh.vertices = newVertices;
        mesh.triangles = triangles;
        mesh.uv = uvs;
        mesh.RecalculateNormals();
        if (bumpMapCompatible)
        {
            mesh.RecalculateTangents();
        }
        mesh.UploadMeshData(false);
    }

    void OnCollisionEnter(Collision collision)
    {
        Debug.Log("Collision detected at " + collision.transform.position + " with impulse " + collision.impulse);

        int numContacts = collision.contactCount;

        ContactPoint[] contactPoints = new ContactPoint[numContacts];
            
        collision.GetContacts(contactPoints);

        //We don't want a collision with multiple contact points to be treated as multiple collisions, so we'll multiply the impulse at each contact point by a value that reflects the number of contact points
        float contactPointsMultiplier = 1.0f / (float)numContacts;

        
        Vector3 localImpulse = transform.InverseTransformVector(collision.impulse) * -1.0f * contactPointsMultiplier;

        for (int i=0; i < points.Length; i++)
        {
            foreach (ContactPoint contactPoint in contactPoints)
            {
                Vector3 localPosition = transform.InverseTransformPoint(contactPoint.point);
                float distance = Vector3.Distance(points[i].position, localPosition);
                forceAccumulator[i] += localImpulse / (distance * distance);
                Debug.Log(forceAccumulator[i]);
            }
        }

        Debug.Log("Contact points: " + collision.contactCount);
    }

    // Update is called once per frame
    void Update()
    {
        if (!mesh)
            return;
        AccumulateForces();
        Verlet();
        SatisfyConstraints();

        UpdateMesh();
    }
}