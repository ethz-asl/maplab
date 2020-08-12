import extract
from datasets.lidar_dataset import LidarSynthetic
import numpy as np


class Args:
    def __init__(self):
        self.model = None
        self.top_k = 300
        self.scale_f = 2 ** 0.25
        self.min_size = 1024
        self.max_size = 2048
        self.min_scale = 1
        self.max_scale = 1

        self.reliability_thr = 0.7
        self.repeatability_thr = 0.7

        self.gpu = -1


# def execute_global_registration(source_down, target_down, reference_desc, target_desc, distance_threshold):
#     result = o3d.registration.registration_ransac_based_on_feature_matching(
#         source_down, target_down, reference_desc, target_desc,
#         distance_threshold,
#         o3d.registration.TransformationEstimationPointToPoint(False), 4,
#         [o3d.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold),
#          o3d.registration.CorrespondenceCheckerBasedOnEdgeLength(0.05)],
#         o3d.registration.RANSACConvergenceCriteria(80000, 500))
#     return result


def remove_invalid_keypoints(xys, scores, desc, mask):
    valid_mask = mask[xys[:, 1].astype(int), xys[:, 0].astype(int)]
    xys = xys[valid_mask]
    scores = scores[valid_mask]
    desc = desc[valid_mask]
    return xys, scores, desc


# def draw_registration_result(source, target, transformation):
#     import copy
#     source_temp = copy.deepcopy(source)
#     target_temp = copy.deepcopy(target)
#     source_temp.paint_uniform_color([1, 0.706, 0])
#     target_temp.paint_uniform_color([0, 0.651, 0.929])
#     source_temp.transform(transformation)
#     o3d.visualization.draw_geometries([source_temp, target_temp])


def test(a):
    # initialize arguments for feature extraction
    args = Args()
    args.model = "/home/dominic/maplab_ws/src/maplab_experimental/maplab/console-plugins/dense-mapping-constraints-plugin/src/pFiles/test.pt"
    show = False
    # initialize lidar db
    root = '/media/dominic/Extreme SSD/datasets/asl_koze/data_both_side/data'
    db = LidarSynthetic(root, skip=(0, -1, 1), crop=False)
    idx1 = 45
    idx2 = idx1 + 98
    # initialize pose
    pose = [np.array([0, 0, 0])]

    imgA = db.get_image(idx1)
    imgB = db.get_image(idx2)

    maskA = db.get_valid_range_mask(idx1)
    xysA, scoresA, descA = extract.extract_keypoints(imgA, args)
    xysA, scoresA, descA = remove_invalid_keypoints(xysA, scoresA, descA, maskA)
    xyzA = db.get_xyz(idx1)
    xyzA_sort = xyzA[xysA[:, 1].astype(int), xysA[:, 0].astype(int)]
    xyzA = xyzA.reshape((-1, 3))

    maskB = db.get_valid_range_mask(idx2)
    xysB, scoresB, descB = extract.extract_keypoints(imgB, args)
    xysB, scoresB, descB = remove_invalid_keypoints(xysB, scoresB, descB, maskB)
    xyzB = db.get_xyz(idx2)
    xyzB_sort = xyzB[xysB[:, 1].astype(int), xysB[:, 0].astype(int)]
    xyzB = xyzB.reshape((-1, 3))

    # reference_pc = o3d.geometry.PointCloud()
    # reference_pc.points = o3d.utility.Vector3dVector(xyzA)
    #
    # test_pc = o3d.geometry.PointCloud()
    # test_pc.points = o3d.utility.Vector3dVector(xyzB)
    #
    # ref = o3d.registration.Feature()
    # ref.data = descA.T
    #
    # test = o3d.registration.Feature()
    # test.data = descB.T
    #
    # print(ref.dimension(), ref.num())
    # print(test.dimension(), test.num())
    #
    # ref_key = o3d.geometry.PointCloud()
    #
    # ref_key.points = o3d.utility.Vector3dVector(xyzA_sort)
    # test_key = o3d.geometry.PointCloud()
    # test_key.points = o3d.utility.Vector3dVector(xyzB_sort)
    #
    # result_ransac = execute_global_registration(ref_key, test_key, ref, test, 0.75)
    # print(result_ransac)
    # print(result_ransac.transformation)
    # tf = result_ransac.transformation
    # matches = np.array(result_ransac.correspondence_set)
    #
    # # # Plot point clouds after registration
    # # draw_registration_result(reference_pc, test_pc, np.eye(4))
    # # draw_registration_result(reference_pc, test_pc, tf)
    #
    # # show results
    # if show:
    #     def blended(xys, img, matches):
    #         x = xys[matches, 0].astype(int)
    #         y = xys[matches, 1].astype(int)
    #         r, i, s = img.split()
    #         i = np.array(i)
    #         i = cv2.cvtColor(i, cv2.COLOR_GRAY2RGB)
    #         # i = np.array(img)
    #         for k in range(x.shape[0]):
    #             # if not mask[k]: continue
    #             i = cv2.circle(i, (x[k], y[k]), 2, (0, 0, 255), 1)
    #         return i
    #     db.inpaint = False
    #     # imgA, imgB, _ = db.get_pair(idx)
    #     blendA = blended(xysA, imgA, matches[:, 0].astype(int))
    #     blend = blended(xysB, imgB, matches[:, 1].astype(int))
    #     stacked = np.vstack((blendA, blend))
    #
    #     thickness = 1
    #     lineType = cv2.LINE_AA
    #     for j in range(matches.shape[0]):
    #         x1 = xysA[matches[:, 0].astype(int), 0][j]
    #         y1 = xysA[matches[:, 0].astype(int), 1][j]
    #         x2 = xysB[matches[:, 1].astype(int), 0][j]
    #         y2 = xysB[matches[:, 1].astype(int), 1][j] + 64
    #         # if mask[j]:
    #         #     color = (0, 255, 0)
    #         # else:
    #         #     color = (0, 0, 255)
    #         #     # continue
    #         color = (0, 255, 0)
    #         cv2.line(stacked, (x1, y1), (x2, y2), color, thickness, lineType)
    #
    #     win_inp = 'Keypoints'
    #     cv2.namedWindow(win_inp)
    #     cv2.imshow(win_inp, stacked)
    #     plt.show()
    #     cv2.waitKey(0) & 0xFF
    return {"keypointsA": xyzA_sort, "keypointsB": xyzB_sort, "descriptorsA": descA, "descriptorsB": descB}

# if __name__ == '__main__':
#     ts = 1575642837062210048
#     test(ts)