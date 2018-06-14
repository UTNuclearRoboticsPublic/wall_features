
# Size parameters 
int32 						height 					# Height in pixels of image
int32 						width 					# Width in pixels of image
float32 					pixel_height			# Height in meters of one pixel (in real world)
float32 					pixel_width 			# Width in meters of one pixel (in real world)

# Actual images
sensor_msgs/Image 			rgb_image 				# Simple RGB data (3 x 8bit uchar) 						
sensor_msgs/Image 			depth_image 			# Offset in wall from expected plane (3 x 32bit float)
sensor_msgs/Image 			angle_image 			# Local normals of wall, as angles in X and Y (2 x 8bit uchar)
sensor_msgs/Image 			intensity_image 		# Intensity of LIDAR returns (1 x 8bit uchar)
sensor_msgs/Image 			holistic_image 			# Contains all the above information (prob 9 x 8bit uchar)  <- is this useful? tricky to encode depth as uchar? 

# Other 
std_msgs/Header 			header 					# Frame, Timestamp
geometry_msgs/Transform 	transform 				# Location of wall plane in parent frame