#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "my_nav_msgs/OccupancyGrid3d.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include <math.h>

#define OCCUPIED	100
#define UNOCCUPIED	0

namespace my_nav_msgs {

	void convertOccupancyGrid3dToPointCloud( const my_nav_msgs::OccupancyGrid3d &grid3d, sensor_msgs::PointCloud &cloud )
	{
		cloud.header = grid3d.header;
		int index[grid3d.data.size()];

		int size = 0;
		for(int i = 0; i < grid3d.data.size(); i++)
		{
			if(grid3d.data[i] == OCCUPIED)
			{
				index[size] = i;
				size++;
			}
		}
		cloud.points.resize(size);

		double resol = grid3d.info.resolution;
		uint32_t width = grid3d.info.width;
		uint32_t depth = grid3d.info.depth;
		uint32_t height = grid3d.info.height;
		if(size > 0)
		{
			for(int i = 0; i < size; i++)
			{
				int j = index[i];
				int ix = j % width;
				int iy = (j / width) % depth;
				int iz = j / width / depth;
				cloud.points[i].x = resol/2 + resol*ix + grid3d.info.origin.x;
				cloud.points[i].y = resol/2 + resol*iy + grid3d.info.origin.y;
				cloud.points[i].z = resol/2 + resol*iz + grid3d.info.origin.z;		
			}
		}
	}

	void convertOccupancyGrid3dToPointCloud2( const my_nav_msgs::OccupancyGrid3d &grid3d, sensor_msgs::PointCloud2 &cloud2 )
	{
		sensor_msgs::PointCloud cloud;
		convertOccupancyGrid3dToPointCloud(grid3d, cloud);
		sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2);		
	}

    void convertPointCloudToOccupancyGrid3d( const double resolution, const sensor_msgs::PointCloud &cloud, my_nav_msgs::OccupancyGrid3d &grid3d )
	{
		grid3d.header = cloud.header;
		int points_size = cloud.points.size();
		if(points_size > 0)
		{
			grid3d.info.resolution = resolution;

			double x_min = cloud.points[0].x;
            double y_min = cloud.points[0].y;
            double z_min = cloud.points[0].z;
            double x_max = cloud.points[0].x;
            double y_max = cloud.points[0].y;
            double z_max = cloud.points[0].z;

            for(int i = 1; i < points_size; i++)
            {
                if(x_min > cloud.points[i].x)   x_min = cloud.points[i].x;
                if(y_min > cloud.points[i].y)   y_min = cloud.points[i].y;
                if(z_min > cloud.points[i].z)   z_min = cloud.points[i].z;
                if(x_max < cloud.points[i].x)   x_max = cloud.points[i].x;
                if(y_max < cloud.points[i].y)   y_max = cloud.points[i].y;
                if(z_max < cloud.points[i].z)   z_max = cloud.points[i].z;
            }
            grid3d.info.origin.x = x_min;
            grid3d.info.origin.y = y_min;
            grid3d.info.origin.z = z_min;

            grid3d.info.width = (uint32_t)((x_max - x_min)/resolution) + 1;
            grid3d.info.depth = (uint32_t)((y_max - y_min)/resolution) + 1;
            grid3d.info.height = (uint32_t)((z_max - z_min)/resolution) + 1;

            int size = grid3d.info.width * grid3d.info.depth * grid3d.info.height;
			//ROS_INFO("grid3d size = %d", size);//debug
			grid3d.data.resize(size);
			for(int i = 0; i < points_size; i++)
            {
				int ix = (int)((cloud.points[i].x - grid3d.info.origin.x)/resolution);
				int iy = (int)((cloud.points[i].y - grid3d.info.origin.y)/resolution);
				int iz = (int)((cloud.points[i].z - grid3d.info.origin.z)/resolution);
				
				int index = ix + iy * grid3d.info.width + iz * grid3d.info.width * grid3d.info.depth;
				if(index < size)
				{
					grid3d.data[index] = OCCUPIED;
				}
			}
        }
	}

	void convertPointCloud2ToOccupancyGrid3d( const double resolution, const sensor_msgs::PointCloud2 &cloud2, my_nav_msgs::OccupancyGrid3d &grid3d )
	{
		sensor_msgs::PointCloud cloud;
		sensor_msgs::convertPointCloud2ToPointCloud(cloud2, cloud);	
		convertPointCloudToOccupancyGrid3d(resolution, cloud, grid3d);
	}

	void convertPointCloudToGridData( const double resolution, const sensor_msgs::PointCloud &cloud_in, sensor_msgs::PointCloud &cloud_out )
	{
		my_nav_msgs::OccupancyGrid3d grid3d;
		convertPointCloudToOccupancyGrid3d(resolution, cloud_in, grid3d);
		convertOccupancyGrid3dToPointCloud(grid3d, cloud_out);
	}

	void convertPointCloud2ToGridData( const double resolution, const sensor_msgs::PointCloud2 &cloud2_in, sensor_msgs::PointCloud2 &cloud2_out )
	{
		my_nav_msgs::OccupancyGrid3d grid3d;
		sensor_msgs::PointCloud cloud_in, cloud_out;
		sensor_msgs::convertPointCloud2ToPointCloud(cloud2_in, cloud_in);
		convertPointCloudToGridData(resolution, cloud_in, cloud_out);
		sensor_msgs::convertPointCloudToPointCloud2(cloud_out, cloud2_out);
	}

}
