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


# RANSAC plane segmentation


# Extract inliers

# Save pcd for table
# pcl.save(cloud, filename)


# Extract outliers


# Save pcd for tabletop objects


