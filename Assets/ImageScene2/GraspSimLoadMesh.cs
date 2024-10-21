using System.Collections.Generic;
using System.Collections;
using UnityEngine;
using System.Linq;
using RFUniverse;
using RFUniverse.Attributes;
using System;

#if UNITY_EDITOR
using UnityEditor;
#endif
public class GraspSimLoadMesh : MonoBehaviour
{
    public bool pause = false;
    List<Vector3> allPoints = new();
    List<Quaternion> allQuaternions = new();
    List<List<float>> listJoint = new();
    List<bool> success = new();
    List<List<float>> grabJointState = new();
    int parallelCount;
    List<Transform> envs = new();
    List<ControllerAttr> grippers = new();
    List<RigidbodyAttr> targets = new();

    Vector2Int wh = new Vector2Int(2, 2);
    bool grasp;
    bool gravity;

    int jointCount
    {
        get
        {
            switch (gripperName)
            {
                case "shadow":
                    return 22;
                case "shadowhand":
                    return 23;
                case "barrett":
                    return 4;
                case "svh":
                default:
                    return 9;
            }
        }
    }
    Vector3 gripperRotation
    {
        get
        {
            switch (gripperName)
            {
                case "shadow":
                case "shadowhand":
                    return new Vector3(0, 0, -90);
                case "barrett":
                    return new Vector3(-180, 0, 0);
                case "svh":
                default:
                    return new Vector3(0, 0, 90);
            }
        }
    }
    private void Start()
    {
        PlayerMain.Instance.AddListenerObject("Test", (msg) => StartCoroutine(StartGraspTest(msg)));
    }
    string gripperName;
    IEnumerator StartGraspTest(object[] data)
    {
        Debug.Log("StartGraspTest");
        allPoints.Clear();
        allQuaternions.Clear();
        string meshPath = (string)data[0];
        gripperName = (string)data[1];
        List<float> pose = data[2].ConvertType<List<float>>();
        List<float> joint = data[3].ConvertType<List<float>>();
        parallelCount = (int)data[4];
        grasp = (bool)data[5];
        gravity = (bool)data[6];
        List<List<float>> listPose = RFUniverseUtility.ListSlicer(pose, 16);
        foreach (var item in listPose)
        {
            Matrix4x4 matrix = RFUniverseUtility.ListFloatToMatrix(item);

            Vector3 point = matrix.transpose.GetPosition();
            point = new Vector3(-point.x, point.y, point.z);

            Quaternion quaternion = matrix.transpose.rotation;
            quaternion = new Quaternion(quaternion.x, -quaternion.y, -quaternion.z, quaternion.w);
            quaternion = quaternion * Quaternion.AngleAxis(-90, Vector3.up);
            quaternion = quaternion * Quaternion.AngleAxis(-90, Vector3.forward);

            allPoints.Add(point);
            allQuaternions.Add(quaternion);
        }
        for (int i = 0; i < joint.Count; i++)
        {
            joint[i] *= Mathf.Rad2Deg;
        }
        listJoint = RFUniverseUtility.ListSlicer(joint, jointCount);
        success.Clear();
        grabJointState.Clear();
        envs.Clear();
        grippers.Clear();
        targets.Clear();

        //创建初始环境
        Transform env = new GameObject($"Env0").transform;
        env.SetParent(transform);
        env.localPosition = Vector3.zero;
        envs.Add(env);
        var target = PlayerMain.Instance.LoadMesh(856924, meshPath, true, "Convex");
        if (target == null)
            Debug.LogError("No Mesh");
        target.transform.position = Vector3.zero;
        target.transform.rotation = Quaternion.identity;

        target.Rigidbody.mass = 0.1f;
        target.Rigidbody.sleepThreshold = 0;
        target.gameObject.AddComponent<CollisionState>();
        target.transform.SetParent(env);
        targets.Add(target);

        ControllerAttr newGripper = PlayerMain.Instance.InstanceObject<ControllerAttr>(gripperName, 987654, false);
        newGripper.Init();
        newGripper.transform.SetParent(env);
        grippers.Add(newGripper);
        newGripper.SetTransform(true, true, false, Vector3.up, gripperRotation, Vector3.zero, false);

        StartCoroutine(GraspSim());

        yield break;
    }
    IEnumerator GraspSim()
    {
        yield return RFUniverseUtility.WaitFixedUpdateFrame();

        PlayerMain.Instance.GroundActive = false;
        //复制并行环境
        int wCount = Mathf.FloorToInt(Mathf.Sqrt(parallelCount));
        for (int i = 1; i < parallelCount; i++)
        {
            Transform newEnv = new GameObject($"Env{i}").transform;
            newEnv.SetParent(transform);
            int w = i % wCount;
            int h = i / wCount;
            newEnv.localPosition = new Vector3(w * wh.x, 0, h * wh.y);
            envs.Add(newEnv);

            RigidbodyAttr newTarget = Instantiate(targets[0], newEnv);
            targets.Add(newTarget);

            ControllerAttr newGripper = Instantiate(grippers[0], newEnv);
            newGripper.Init();
            newGripper.SetTransform(true, true, false, grippers[0].transform.localPosition, grippers[0].transform.localRotation.eulerAngles, Vector3.zero, false);
            grippers.Add(newGripper);
        }
        yield return RFUniverseUtility.WaitFixedUpdateFrame();
        //开始循环
        for (int i = 0; i < allPoints.Count; i += parallelCount)
        {
            Debug.Log($"{i}/{allPoints.Count}");
            yield return RFUniverseUtility.WaitFixedUpdateFrame();
            //初始化手和物体
            for (int j = 0; j < parallelCount; j++)
            {
                List<float> currentListJoint = (new float[jointCount]).ToList();
                grippers[j].SetJointPosition(currentListJoint, ControlMode.Direct);
            }
            yield return RFUniverseUtility.WaitFixedUpdateFrame();

            List<Vector3> startPositions = new List<Vector3>();
            List<Quaternion> startRotations = new List<Quaternion>();
            Physics.gravity = Vector3.zero;
            //设置gripper
            for (int j = 0; j < parallelCount; j++)
            {
                if (i + j >= allPoints.Count) break;
                List<float> currentListJoint = listJoint[i + j];

                for (int k = 0; k < currentListJoint.Count; k++)
                {
                    if (float.IsNaN(currentListJoint[k]))
                        currentListJoint[k] = 0;
                }
                grippers[j].SetJointPosition(currentListJoint, ControlMode.Direct);
            }
            yield return RFUniverseUtility.WaitFixedUpdateFrame(1);

            //设置物体位置
            for (int j = 0; j < parallelCount; j++)
            {
                if (i + j >= allPoints.Count) break;
                Vector3 point = allPoints[i + j];
                Quaternion quaternion = allQuaternions[i + j];
                Transform localEnv = envs[j];
                RigidbodyAttr localTarget = targets[j];
                ControllerAttr localGripper = grippers[j];
                localTarget.Rigidbody.isKinematic = true;
                foreach (var item in localTarget.GetComponentsInChildren<Collider>())
                {
                    item.isTrigger = true;
                }
                if (localTarget.GetComponent<CollisionState>() == null)
                    localTarget.gameObject.AddComponent<CollisionState>();
                localTarget.gameObject.GetComponent<CollisionState>().collision = false;

                Transform graspPoint = new GameObject("GraspPoint").transform;
                graspPoint.SetParent(localTarget.transform);
                graspPoint.localPosition = point;
                graspPoint.localRotation = quaternion;
                graspPoint.SetParent(localEnv);
                localTarget.transform.SetParent(graspPoint);

                graspPoint.position = localGripper.transform.position;
                graspPoint.rotation = localGripper.transform.rotation;
                localTarget.transform.SetParent(localEnv);
                Destroy(graspPoint.gameObject);
                startPositions.Add(localTarget.transform.position);
                startRotations.Add(localTarget.transform.rotation);
            }
            yield return RFUniverseUtility.WaitFixedUpdateFrame(1);
            for (int j = 0; j < parallelCount; j++)
            {
                if (i + j >= allPoints.Count) break;
                targets[j].transform.position -= Vector3.up;
            }
            yield return RFUniverseUtility.WaitFixedUpdateFrame();
            for (int j = 0; j < parallelCount; j++)
            {
                if (i + j >= allPoints.Count) break;
                targets[j].transform.position = startPositions[j];
            }
            yield return RFUniverseUtility.WaitFixedUpdateFrame(3);

            bool[] envSuccess = new bool[parallelCount];
            List<float>[] envJointState = new List<float>[parallelCount];
            bool[] envPressed = new bool[parallelCount];

            for (int s = 0; s < envSuccess.Length; s++)
            {
                envSuccess[s] = true;
            }
            //初始状态检查
            for (int j = 0; j < parallelCount; j++)
            {
                if (i + j >= allPoints.Count) break;
                //检测碰撞状态
                envSuccess[j] &= !targets[j].GetComponentInChildren<CollisionState>().collision;
                //if (!envSuccess[j])
                //    Debug.Log("穿插检测失败");
            }


            if (grasp)
            {
                //设置物体Trigger
                for (int j = 0; j < parallelCount; j++)
                {
                    if (!envSuccess[j]) continue;
                    if (i + j >= allPoints.Count) break;
                    foreach (var item in targets[j].GetComponentsInChildren<Collider>())
                    {
                        item.isTrigger = false;
                    }
                }
                //闭合gripper开始抓取
                int endCount = 0;
                for (int j = 0; j < parallelCount; j++)
                {
                    if (!envSuccess[j])
                    {
                        endCount++;
                        continue;
                    }
                    if (i + j >= allPoints.Count)
                    {
                        endCount++;
                        continue;
                    }
                    StartCoroutine(grippers[j].GetComponent<close>().Close(() => endCount++));
                }
                yield return new WaitUntil(() => endCount == parallelCount);

                //保存抓住时的JointState

                for (int j = 0; j < parallelCount; j++)
                {
                    if (i + j >= allPoints.Count) break;
                    envJointState[j] = grippers[j].GetJointPositions();
                }
                //设置抓取目标
                for (int j = 0; j < parallelCount; j++)
                {
                    if (!envSuccess[j]) continue;
                    if (i + j >= allPoints.Count) break;
                    List<float> currentListJoint = grippers[j].GetJointPositions();
                    for (int k = 0; k < currentListJoint.Count; k++)
                    {
                        switch (gripperName)
                        {
                            case "shadow":
                                if (k == 0 || k == 4 || k == 8 || k == 13 || k == 17) continue;
                                currentListJoint[k] += 5;
                                break;
                            case "shadowhand":
                                if (k == 0 || k == 1 || k == 5 || k == 9 || k == 14 || k == 18) continue;
                                currentListJoint[k] += 5;
                                break;
                            case "barrett":
                                if (k == 1) continue;
                                currentListJoint[k] += 5;
                                break;
                            case "svh":
                                if (k == 0 || k == 7) continue;
                                currentListJoint[k] += 5;
                                break;
                        }
                    }
                    grippers[j].SetJointPosition(currentListJoint);
                }
                //设置物体刚体
                for (int j = 0; j < parallelCount; j++)
                {
                    if (!envSuccess[j]) continue;
                    if (i + j >= allPoints.Count) break;
                    targets[j].Rigidbody.isKinematic = false;
                }
                yield return RFUniverseUtility.WaitFixedUpdateFrame(50);

                if (gravity)
                {
                    //施加重力
                    Physics.gravity = Vector3.down * 10;
                    yield return RFUniverseUtility.WaitFixedUpdateFrame(100);

                }
                //判断结束状态
                for (int j = 0; j < parallelCount; j++)
                {
                    if (!envSuccess[j]) continue;
                    if (i + j >= allPoints.Count) break;
                    envSuccess[j] &= Vector3.Distance(targets[j].transform.position, startPositions[j]) < 0.1f;
                    //envSuccess[j] &= (targets[j].Rigidbody.velocity.sqrMagnitude < 0.01f);
                    //if (!envSuccess[j])
                    //    Debug.Log("重力检测失败");
                }
            }
            //写入结果
            for (int j = 0; j < parallelCount; j++)
            {
                if (i + j >= allPoints.Count) break;
                success.Add(envSuccess[j]);
                grabJointState.Add(envJointState[j]);
                //Debug.Log(envSuccess[j]);
            }
        }
        //清理
        for (int j = 0; j < parallelCount; j++)
        {
            List<float> currentListJoint = (new float[23]).ToList();
            grippers[j].SetJointPosition(currentListJoint, ControlMode.Direct);
        }
        yield return RFUniverseUtility.WaitFixedUpdateFrame();
        for (int i = 0; i < parallelCount; i++)
        {
            Destroy(envs[i].gameObject);
        }
        yield return RFUniverseUtility.WaitFixedUpdateFrame();
        GC.Collect();
        Debug.Log("Done");
        PlayerMain.Instance.SendObject("Result", success, grabJointState);
    }
}
