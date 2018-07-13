import Augmentor

# # Training Data
# for iCluster in range(1, 13):
#
#     p = Augmentor.Pipeline("E:/works/Topics/CNN_Robot/image_3.28/train/"+str(iCluster))
#
#     p.crop_by_size(probability=1.0, width=400, height=400, centre=False)
#     p.rotate(probability=0.7, max_left_rotation=10, max_right_rotation=10)
#     p.zoom(probability=0.5, min_factor=1.0, max_factor=1.5)
#     p.flip_left_right(probability=0.5)
#
#     p.sample(2000)

# Test Data
for iCluster in range(1, 5):
    #p = Augmentor.Pipeline("E:/works/Topics/CNN_Robot/image_3.28/test/"+str(iCluster))
    p = Augmentor.Pipeline("D:/RobotVision/image_3.28/train/"+str(iCluster))
    p.crop_by_size(probability=1.0, width=400, height=400, centre=False)

    p.sample(100)
