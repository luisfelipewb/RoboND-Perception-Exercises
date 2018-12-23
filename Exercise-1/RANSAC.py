# Import PCL module
import pcl

# Load Point Cloud file
cloud = pcl.load_XYZRGB('tabletop.pcd')

# Voxel Grid filter
vox = cloud.make_voxel_grid_filter()

# EVALUATION
#sizes = [1, 0.1, 0.01, 0.001, 0.0001]
#for LEAF_SIZE in sizes: 
#	vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
#	cloud_filtered = vox.filter()
#	filename = 'voxel_downsampled_'+str(LEAF_SIZE)+'.pcd'
#	pcl.save(cloud_filtered, filename)
#
# RESULTS
# 9.6M	tabletop.pcd
# 9.6M	voxel_downsampled_0.0001.pcd	Already maximum resolution
# 9.6M	voxel_downsampled_0.001.pcd	Already maximum resolution
# 2.7M	voxel_downsampled_0.01.pcd	Decreases more than 60% of file size
# 44K	voxel_downsampled_0.1.pcd	Bad resolution
# 4.0K	voxel_downsampled_1.pcd 	Very bad resolution

LEAF_SIZE = 0.01 #unit is meters
vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
cloud_filtered = vox.filter()
filename = 'voxel_downsampled.pcd'
pcl.save(cloud_filtered, filename)

# PassThrough filter
passthrough = cloud_filtered.make_passthrough_filter()

filter_axis = 'z'
passthrough.set_filter_field_name(filter_axis)
axis_min = 0.74
axis_max = 1.1
passthrough.set_filter_limits(axis_min, axis_max)

cloud_filtered = passthrough.filter()
filename = 'pass_through_filtered.pcd'
pcl.save(cloud_filtered, filename)

# RANSAC plane segmentation
seg = cloud_filtered.make_segmenter()

seg.set_model_type(pcl.SACMODEL_PLANE)
seg.set_method_type(pcl.SAC_RANSAC)

max_distance = 0.02
seg.set_distance_threshold(max_distance)
inliers, coefficients = seg.segment()

# Extract inliers
extracted_inliers = cloud_filtered.extract(inliers, negative=False)
filename = 'ransac_inliers.pcd'
pcl.save(extracted_inliers, filename)

# Extract outliers
extracted_outliers = cloud_filtered.extract(inliers, negative=True)
filename = 'ransac_outliers.pcd'
pcl.save(extracted_outliers, filename)

cloud_filtered = extracted_outliers

# Outlier Removal Filter (not needed in this example)

outlier_filter = cloud_filtered.make_statistical_outlier_filter()
outlier_filter.set_mean_k(50)
x = 1.0
outlier_filter.set_std_dev_mul_thresh(x)

cloud_filtered = outlier_filter.filter()
filename = 'outlier_filtered.pcd'
pcl.save(cloud_filtered, filename)

