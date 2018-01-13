#ifndef _VRMLPARSER_H_
#define _VRMLPARSER_H_

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <fstream>
#include <vector>
#include <random>
#include <vector>

#include <mutex>

#include <bullet/Actor.h>

#include <json/json.h>
#include <json/value.h>
#include <json/reader.h>
#include <json/writer.h>

struct Body
{
    std::string url;
    std::string collision_url;
    std::string name;
    Eigen::Matrix4d transform;
};

struct Model
{
    std::string name;
    std::string url;
    std::vector<Body> bodies;
    Eigen::Vector3d bbox;
};

struct SceneGraph{
    std::string url;
    std::vector<Model> models;
};

static Eigen::MatrixXd readJsonMatrix(Json::Value Array){
    int rows = Array.size();
    int cols = Array[0].size();

    Eigen::MatrixXd Matrix(rows,cols) ;

    for(int i = 0 ; i<rows ; i++){
        for(int j=0 ; j<cols ; j++){
            Matrix(i,j) = Array[i][j].asDouble() ;
        }
    }
    return Matrix;
}

static Eigen::VectorXd readJsonVector(Json::Value Array){
   int size = Array.size();

   Eigen::VectorXd Vector(size);
   for(int i = 0 ; i < size ; ++i){
       Vector(i) = Array[i].asDouble();
   }
   return Vector;
}

class SGJSONParser
{
public:

    SGJSONParser();

    std::vector<bpp_actor::Actor> loadScene(const std::string& json_string);

    bool loadActors(const std::string json_string, std::vector<bpp_actor::Actor> &actors);

    bool saveActors(std::string &json_string, std::vector<bpp_actor::Actor> &actors);

    std::string generateWRLFile(bpp_actor::Actor &box);

    bool saveBoxPlan(std::string &json_string, std::vector<bpp_msgs::BoxPlan> &boxplan_vec);

    bool loadBoxPlan(const std::string json_string, std::vector<bpp_msgs::BoxPlan> &boxplan_vec);

    bool loadActors(Json::Value box_list, std::vector<bpp_actor::Actor> &actors);

    bool saveActors(Json::Value &box_list, std::vector<bpp_actor::Actor> &actors);

    bool saveBoxPlan(Json::Value &json_val, std::vector<bpp_msgs::BoxPlan> &boxplan_vec);

    bool loadBoxPlan(Json::Value boxplan_list, std::vector<bpp_msgs::BoxPlan> &boxplan_vec);

};

#endif // _VRMLPARSER_H_
