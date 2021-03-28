// Read in points, normals and the transformation matrix from the .dae file
// Build the packages in this project repository (replace `<your_catkin_ws>` with your catkin workspace name) with:
// cd ~/<your_catkin_ws>/src/mobile-robotic-manipulation
// CMAKE_PREFIX_PATH=~/<your_catkin_ws>/devel:/opt/ros/melodic catkin build motion_planning
// Source your workspace
// cd /<your_catkin_ws>/src
// In terminal 1, run:
// roscore &
// In terminal 2, run:
// rosrun motion_planning motion_planning & rosrun rviz rviz
// To see the markers in RViz click Add->Markers 

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tinyxml.h>
#include <iostream>
#include <string>
#include <sstream>

std::vector<float> x_values;
std::vector<float> y_values;
std::vector<float> z_values;

std::vector<float> x_normal;
std::vector<float> y_normal;
std::vector<float> z_normal;

float rot_matrix[4][4];

int main( int argc, char** argv )
{

  ros::init(argc, argv, "basic_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Start reading the .dae file
  TiXmlDocument doc( "mobile-robotic-manipulation/motion_planning/boat/meshes/boat.dae" );
  bool check_file = doc.LoadFile();
  // Check if loading the .dae file works
  if (check_file)
  {
    std::cout << "Open" << std::endl;
  } else {
    std::cout << "Failed to load file" << std::endl;
  }
  int size_points;
  int size_total; 
  TiXmlElement *l_pRootElement = doc.RootElement();
 if( NULL != l_pRootElement )
  {
   	// Read the points
    TiXmlElement *l_library_geometries = l_pRootElement->FirstChildElement( "library_geometries" );
    if ( NULL != l_library_geometries )
    {
      TiXmlElement *l_geometry = l_library_geometries->FirstChildElement( "geometry" );
      if( l_geometry )
      {
        TiXmlElement *l_mesh = l_geometry->FirstChildElement( "mesh" );
        if( l_mesh )
        {
          TiXmlElement *l_source = l_mesh->FirstChildElement( "source" );

          for(TiXmlElement* l_source = l_mesh->FirstChildElement("source"); l_source != NULL; l_source = l_source->NextSiblingElement("source"))
          {
            const char *attributeOfSource = l_source->Attribute("id");

            if( strcmp(attributeOfSource,"riva_1-mesh-positions")==0 )
            {

              TiXmlElement *l_float_array = l_source->FirstChildElement( "float_array" );
              if ( NULL != l_float_array )
              {
                const char *attributeOffloat_array = l_float_array->Attribute("id");
                const char *attributeOffloat_array_count = l_float_array->Attribute("count");
		            size_total = strtol(attributeOffloat_array_count,NULL,10);
                const char* points_str = l_float_array->GetText();

                size_points=size_total/3;

                x_values.resize(size_points);
                y_values.resize(size_points);
                z_values.resize(size_points);

                std::stringstream points_stream(points_str);   
                float data[size_total];
                for(int i=0;i<size_total;i++)
                {
                  points_stream >> data[i];
                  for(int i=0;i<size_total;i++)
                  {
                    if ((i)%3 == 0) {
                      x_values[i/3]=data[i];
                    }
                    if ((i)%3 == 1) {
                      y_values[i/3]=data[i];
                    }
                    if ((i)%3 == 2) {
                      z_values[i/3]=data[i];
                    }
                  }
                }
              }
            }
                  
            // Read the normals
            if( strcmp(attributeOfSource,"riva_1-mesh-normals")==0 )
            {
                     
              TiXmlElement *l_float_array = l_source->FirstChildElement( "float_array" );
              if ( NULL != l_float_array )
              {

                const char* points_str = l_float_array->GetText();

                x_normal.resize(size_points);
                y_normal.resize(size_points);
                z_normal.resize(size_points);
            
                std::stringstream points_stream(points_str);   
                float data[size_total];
                for(int i=0;i<size_total;i++)
                {
                  points_stream >> data[i];
                  for(int i=0;i<size_total;i++)
                  {
                    if ((i)%3 == 0) {
                      x_normal[i/3]=data[i];
                    }
                    if ((i)%3 == 1) {
                      y_normal[i/3]=data[i];
                    }
                    if ((i)%3 == 2) {
                      z_normal[i/3]=data[i];
                    }                    
                  }

                }

              }
            }
          }
        }
      }

    }    

    // Read transformation matrix
    TiXmlElement *l_library_visual_scenes = l_pRootElement->FirstChildElement( "library_visual_scenes" );
    if ( NULL != l_library_visual_scenes )
    {
      TiXmlElement *l_visual_scene = l_library_visual_scenes->FirstChildElement( "visual_scene" );
      if( l_visual_scene )
      {
        TiXmlElement *l_node = l_visual_scene->FirstChildElement( "node" );
        if( l_node )
        {
          TiXmlElement *l_matrix = l_node->FirstChildElement( "matrix" );
          if( NULL != l_matrix )
          {

            const char* matrix_str = l_matrix->GetText();

            std::stringstream matrix_stream(matrix_str);  
               
            for (int i=0;i<4;i++){
              for (int j=0;j<4;j++){                     
                matrix_stream >> rot_matrix[i][j];                             
              }
            }

          }
        }
      }
    } 

  }
    
  // End reading the .dae file


  // Creating markers
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp. See RViz Markers tutorial
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();

  for(int i=0;i<size_points;i++){

    float point_transf[] = { 0, 0, 0, 0 }; 
    float point[] = { x_values[i], y_values[i], z_values[i], 1 };
    for (int i=0;i<4;i++){
      for (int j=0;j<4;j++){
        point_transf[i]+= rot_matrix[i][j]*point[j] ;     
      }
    }  

    // Set the namespace and id for the marker
    marker.ns = "basic_markers";
    marker.id = i;
    // Set the marker type. 
    marker.type = visualization_msgs::Marker::POINTS; 
    // Set the marker action
    marker.action = visualization_msgs::Marker::ADD;
    // Set the scale of the marker 
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    // Set the color 
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    // Set point values
    geometry_msgs::Point p;
    p.x = point_transf[0];
    p.y = point_transf[1];
    p.z = point_transf[2];
    marker.points.push_back(p);
    marker.lifetime = ros::Duration(); 
    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
        sleep(1);
    }
    marker_pub.publish(marker);

  }

}








