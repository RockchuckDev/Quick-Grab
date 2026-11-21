using Godot;
using System;
using System.Collections.Generic;

public partial class GrabManager : Node
{

    public Dictionary<string, GrabData> allActiveGrabData = new Dictionary<string, GrabData>();
    [Export] public float grabStrength;
    [Export] public float grabDampening;
    [Export] public float rotationGrabStrength;
    [Export] public float rotationGrabDampening;


    public GrabData Grab(RigidBody3D rigidBody, Vector3 grabPosition, string id = "null_id", Node3D keepRotationRelativeToThisNode = null, QGRotationHandlingMode rotationHandlingMode = QGRotationHandlingMode.Limp)
    {

        // GRAB POINT________________
        if (rigidBody.GetNodeOrNull<Node3D>("Grab Point") is Node3D grabPointNode)
        {
            grabPosition = grabPointNode.GlobalPosition;
        }

        GrabData grabData = new GrabData();
        grabData.grabbedRigidbody = rigidBody;

        grabData.rotationHandlingMode = rotationHandlingMode;

        //INITIALIZE TARGETS________________
        grabData.targetPosition = grabPosition;
        grabData.targetGrabPivotBasis = Basis.Identity;

        // GRAB PIVOT STUFF________________
        grabData.grabPivot = new RigidBody3D();
        grabData.grabPivot.Position = grabPosition;
        grabData.grabPivot.Mass = 1;
        GetTree().Root.AddChild(grabData.grabPivot);

        // JOINT STUFF________________
        Generic6DofJoint3D joint = new Generic6DofJoint3D();
        joint.Position = grabPosition;
        grabData.joint = joint;
        GetTree().Root.AddChild(grabData.joint);
        joint.NodeA = grabData.grabPivot.GetPath();
        joint.NodeB = rigidBody.GetPath();

        // ID STUFF________________
        string ID = id != "null_id" ? id : $"{allActiveGrabData.Count}";
        grabData.id = ID;
        allActiveGrabData[ID] = grabData;

        // BASIS INITIALIZING________________
        grabData.initialGrabbedObjectBasis = rigidBody.GlobalBasis;
        // We don't need to account for the Basis of the joint, because the joint always starts with an Identity ("no rotation") basis
        grabData.grabbedObjectToGrabPivotBasis = grabData.initialGrabbedObjectBasis.Inverse();

        if(keepRotationRelativeToThisNode != null){
            grabData.initialOffsetBasisGrabbedObjectToNode = keepRotationRelativeToThisNode.GlobalBasis;
            grabData.nodeToKeepRotationRelativeTo = keepRotationRelativeToThisNode;
        }

        return grabData;
    }

    public void Drop(GrabData grabData){
        grabData.grabPivot.QueueFree();
        grabData.joint.QueueFree();
        allActiveGrabData.Remove(grabData.id);
    }


    // Called when the node enters the scene tree for the first time.
    public override void _Ready()
    {
    }

    // Called every frame. 'delta' is the elapsed time since the previous frame.
    public override void _Process(double delta)
    {
        foreach (GrabData grabData in allActiveGrabData.Values)
        {
            // POSITION HANDLING
            Vector3 toTarget = grabData.targetPosition - grabData.grabPivot.GlobalPosition;
            grabData.grabPivot.ApplyForce(toTarget * grabStrength - grabData.grabPivot.LinearVelocity * grabDampening);

            // SET THE TARGET BASIS BASED ON rotationHandlingMode AND ANY RELEVANT METADATA
            switch (grabData.rotationHandlingMode)
            {
                case QGRotationHandlingMode.KeepRotationRelativeToNode:
                    if (grabData.nodeToKeepRotationRelativeTo != null)
                    {
                        Basis targetBasisBeforeConvertedToLocalSpace = grabData.initialOffsetBasisGrabbedObjectToNode;
                        if(grabData.grabbedRigidbody.HasMeta("Rotation_Override")){
                            targetBasisBeforeConvertedToLocalSpace = Basis.FromEuler(grabData.grabbedRigidbody.GetMeta("Rotation_Override").AsVector3().CR_ToRadians());
                        }
                        grabData.targetGrabPivotBasis = grabData.nodeToKeepRotationRelativeTo.GlobalBasis * targetBasisBeforeConvertedToLocalSpace;
                    }
                    else
                    {
                        GD.PrintErr("GrabData with id " + grabData.id + " has KeepRotationRelativeToNode rotation mode, but nodeToKeepRotationRelativeTo is null");
                    }
                    break;
                case QGRotationHandlingMode.KeepRotationInWorldSpace:
                    grabData.targetGrabPivotBasis = grabData.grabbedRigidbody.HasMeta("Rotation_Override") ?
                        Basis.FromEuler(grabData.grabbedRigidbody.GetMeta("Rotation_Override").AsVector3().CR_ToRadians()) :
                        Basis.Identity;
                    break;
            }

            // ROTATION HANDLING
            if(grabData.rotationHandlingMode != QGRotationHandlingMode.Limp){
                Quaternion toTargetBasis = (grabData.targetGrabPivotBasis * grabData.grabbedRigidbody.GlobalBasis.Inverse()).GetRotationQuaternion();
                grabData.grabPivot.ApplyTorque(toTargetBasis.GetAxis() * toTargetBasis.GetAngle() * rotationGrabStrength - grabData.grabPivot.AngularVelocity * rotationGrabDampening);

            }

        }
    }
}


public class GrabData {
    public RigidBody3D grabbedRigidbody;
    public RigidBody3D grabPivot;
    public Generic6DofJoint3D joint;

    public Vector3 targetPosition;

    public Basis targetGrabPivotBasis;
    public Basis initialGrabbedObjectBasis;
    public Basis grabbedObjectToGrabPivotBasis;

    public string id;

    public QGRotationHandlingMode rotationHandlingMode;

    // Keep relative rotation
    public Node3D nodeToKeepRotationRelativeTo;
    public Basis initialOffsetBasisGrabbedObjectToNode;

    public void SetTargetPosition(Vector3 position)
    {
        targetPosition = position;
    }

    public void SetTargetBasis(Basis basis)
    {
        targetGrabPivotBasis = basis;
    }
}

public enum QGRotationHandlingMode
{
    Limp,
    KeepRotationRelativeToNode,
    KeepRotationInWorldSpace
}
