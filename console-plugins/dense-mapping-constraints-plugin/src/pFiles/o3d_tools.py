import open3d as o3d
import numpy as np


def execute_global_registration(source_down, target_down, reference_desc, target_desc, distance_threshold):
    result = o3d.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, reference_desc, target_desc,
        distance_threshold,
        o3d.registration.TransformationEstimationPointToPoint(False), 4,
        [o3d.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold),
         o3d.registration.CorrespondenceCheckerBasedOnEdgeLength(0.05)],
        o3d.registration.RANSACConvergenceCriteria(80000, 500))
    return result


def draw_registration_result(source, target, transformation):
    import copy
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


def get_tf(xyzA, xyzB, descA, descB, xyzA_sort, xyzB_sort, debug):
    reference_pc = o3d.geometry.PointCloud()
    reference_pc.points = o3d.utility.Vector3dVector(xyzA)

    test_pc = o3d.geometry.PointCloud()
    test_pc.points = o3d.utility.Vector3dVector(xyzB)

    ref = o3d.registration.Feature()
    ref.data = descA.T

    test = o3d.registration.Feature()
    test.data = descB.T

    ref_key = o3d.geometry.PointCloud()
    ref_key.points = o3d.utility.Vector3dVector(xyzA_sort)
    test_key = o3d.geometry.PointCloud()
    test_key.points = o3d.utility.Vector3dVector(xyzB_sort)

    print(ref.dimension(), ref.num(), len(ref_key.points))
    print(test.dimension(), test.num(), len(test_key.points))

    result_ransac = execute_global_registration(ref_key, test_key, ref, test, 0.75)

    tf = result_ransac.transformation.tolist()

    # tf = np.array([[-0.926815, -0.372222, -0.0496457, -0.837011],
    #                [0.375364, -0.922095, -0.0940335, 0.55793],
    #                [-0.0107767, -0.105787, 0.994331, 0.0345585],
    #                [0, 0, 0, 1.]], dtype=np.float64)

    if debug:
        print(ref.dimension(), ref.num(), len(ref_key.points))
        print(test.dimension(), test.num(), len(test_key.points))
        draw_registration_result(reference_pc, test_pc, np.eye(4))
        draw_registration_result(reference_pc, test_pc, tf)
        print(result_ransac.transformation)

    # return tf, fitness
    return result_ransac
