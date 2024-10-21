using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class close : MonoBehaviour
{
    public List<ArticulationBody> rootJoints;
    private List<List<ArticulationBody>> movableJoints;
    private List<List<CollisionState>> collisionStates;
    private void Awake()
    {
        movableJoints = new List<List<ArticulationBody>>();
        collisionStates = new List<List<CollisionState>>();
        foreach (var rootJoint in rootJoints)
        {
            ArticulationBody[] childJoints = rootJoint.GetComponentsInChildren<ArticulationBody>();
            List<ArticulationBody> movableChildJoints = new List<ArticulationBody>();
            foreach (var childJoint in childJoints)
                if (childJoint.jointType != ArticulationJointType.FixedJoint) movableChildJoints.Add(childJoint);
            CollisionState[] collision = rootJoint.GetComponentsInChildren<CollisionState>();
            movableJoints.Add(movableChildJoints);
            collisionStates.Add(SetCollision(movableChildJoints));
        }
    }

    private List<CollisionState> SetCollision(List<ArticulationBody> childJoints)
    {
        return childJoints.Select(s => s.GetComponent<CollisionState>()).ToList();
    }

    public IEnumerator Close(Action end)
    {
        float startTime = Time.time;
        bool[] reachTarget = new bool[rootJoints.Count];
        for (int i = 0; i < reachTarget.Length; i++)
        {
            reachTarget[i] = false;
            foreach (var item in rootJoints[i].GetComponentsInChildren<Collider>())
                item.isTrigger = true;
            foreach (var item in collisionStates[i])
                item.collision = false;
        }
        while (true)
        {
            if (Time.time - startTime > 30)
            {
                end.Invoke();
                yield break;
            }
            for (int i = 0; i < reachTarget.Length; i++)
            {
                if (!reachTarget[i])
                {
                    List<ArticulationBody> joints = movableJoints[i];
                    bool allJointReachLimit = true;
                    bool temp_col = false;
                    for (int j = joints.Count - 1; j >= 0; j--)
                    {
                        ArticulationBody joint = joints[j];
                        temp_col |= collisionStates[i][j].collision;
                        if (!temp_col && joint.jointPosition[0] < joint.xDrive.upperLimit * Mathf.Deg2Rad - 0.02)
                        {
                            //joint.jointPosition = new ArticulationReducedSpace(joint.jointPosition[0] + 0.02f);
                            ArticulationDrive drive = joint.xDrive;
                            drive.target += 1;
                            joint.xDrive = drive;
                            allJointReachLimit = false;
                        }
                    }
                    reachTarget[i] = allJointReachLimit;
                    if (reachTarget[i])
                        foreach (var joint in joints)
                        {
                            joint.jointPosition = new ArticulationReducedSpace(joint.jointPosition[0] - 0.02f);
                            ArticulationDrive drive = joint.xDrive;
                            drive.target = joint.jointPosition[0] * Mathf.Rad2Deg;
                            joint.xDrive = drive;
                        }
                }
            }
            yield return new WaitForFixedUpdate();
            bool allReachTarget = true;
            for (int i = 0; i != reachTarget.Length; i++)
                if (!reachTarget[i]) allReachTarget = false;
            if (allReachTarget)
                break;
        }
        for (int i = 0; i < reachTarget.Length; i++)
        {
            foreach (var item in rootJoints[i].GetComponentsInChildren<Collider>())
            {
                item.isTrigger = false;
            }
        }
        end.Invoke();
    }
}