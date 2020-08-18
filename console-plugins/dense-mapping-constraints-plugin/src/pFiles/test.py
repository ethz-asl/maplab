import extract
import numpy as np
from PIL import Image
from o3d_tools import get_tf

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

        self.gpu = [0]


def remove_invalid_keypoints(xys, scores, desc, mask):
    valid_mask = mask[xys[:, 1].astype(int), xys[:, 0].astype(int)]
    xys = xys[valid_mask]
    scores = scores[valid_mask]
    desc = desc[valid_mask]
    return xys, scores, desc


def open_img(path):
    try:
        return np.load(path)
    except Exception as e:
        raise IOError("Could not load image %s (reason: %s)" % (path, str(e)))


def open_lidar_image(folder_path):
    # convert to image
    def img_conv(m):
        im = ((m / 4. + 1.) / 2. * 255).astype(np.uint8)
        return im
    rang = img_conv(open_img(folder_path + '/range.npy'))
    refl = img_conv(open_img(folder_path + '/reflectivity.npy'))
    inte = img_conv(open_img(folder_path + '/intensity.npy'))

    img = np.stack((rang, inte, refl), axis=2)
    img = Image.fromarray(img, mode='RGB')
    return img


def get_xyz(folder_path):
    xyz = np.load(folder_path + '/xyz.npy')
    return xyz


def get_valid_range_mask(folder_path):
    mask = np.load(folder_path + '/valid_mask.npy')
    return mask


def test(ts_pair, debug):
    # initialize arguments for feature extraction
    args = Args()
    args.model = "/home/dominic/CLionProjects/cpp-python-test/pFiles/test.pt"
    # load the network...
    net = extract.load_network(args.model, True)
    # initialize lidar db
    root = '/media/dominic/Extreme SSD/datasets/asl_koze/data_both_side/data/'

    ts1 = str(ts_pair[0])
    ts2 = str(ts_pair[1])
    imgA = open_lidar_image(root + ts1)
    imgB = open_lidar_image(root + ts2)

    maskA = get_valid_range_mask(root + ts1)
    xysA, scoresA, descA = extract.extract_keypoints(imgA, args, net)
    xysA, scoresA, descA = remove_invalid_keypoints(xysA, scoresA, descA, maskA)
    xyzA = get_xyz(root + ts1)
    xyzA_sort = xyzA[xysA[:, 1].astype(int), xysA[:, 0].astype(int)]
    xyzA = xyzA.reshape((-1, 3))

    maskB = get_valid_range_mask(root + ts2)
    xysB, scoresB, descB = extract.extract_keypoints(imgB, args, net)
    xysB, scoresB, descB = remove_invalid_keypoints(xysB, scoresB, descB, maskB)
    xyzB = get_xyz(root + ts2)
    xyzB_sort = xyzB[xysB[:, 1].astype(int), xysB[:, 0].astype(int)]
    xyzB = xyzB.reshape((-1, 3))

    get_tf(xyzA, xyzB, descA, descB, xyzA_sort, xyzB_sort, debug)

    # Show debug info
    if debug:
        # imgA.show()
        # imgB.show()
        print(xyzA_sort.shape, descA.shape)
        get_tf(xyzA, xyzB, descA, descB, xyzA_sort, xyzB_sort, debug)

    return {"keypoints_A": xyzA_sort.tolist(), "keypoints_B": xyzB_sort.tolist(), "descriptors_A": descA.tolist(),
            "descriptors_B": descB.tolist(), "num_points_A": descA.shape[0], "num_points_B": descB.shape[0]}


# if __name__ == '__main__':
# #     ts1 = "1575643036862837760"
# #     ts2 = "1575642966162436096"
# #
# #     out = test([(ts1, ts2), (ts1, ts2), (ts1, ts2), (ts1, ts2)], 1)
#     ts_list = np.loadtxt("/home/dominic/maplab_ws/src/maplab_experimental/maplab/console-plugins/dense-mapping-constraints-plugin/src/pFiles/example.txt"
#                          , delimiter=", ")
#     ts_list = ts_list[::100, :]
#     print(ts_list.shape)
#     for i in range(ts_list.shape[0]):
#         ts_pair = ts_list[i, :].astype(int)
#         test(ts_pair, 1)
