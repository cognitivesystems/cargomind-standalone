#include "SGJSONParser.h"
#include "BlobStoreHelper.h"

SGJSONParser::SGJSONParser()
{
    std::cout << "SGJSONParser" << std::endl;
}

std::vector<bpp_actor::Actor> SGJSONParser::loadScene(const std::string& json_string)
{
    std::cout << "loading scene" << std::endl;
    std::cout << json_string << std::endl;

    Json::Value root;
    Json::Reader reader_json;
    bool read_success = reader_json.parse(json_string, root, false);
    if(!read_success)
    {
        std::cout << "scene graph file couldn't be read" << std::endl;
//        return false;
    }
    Json::Value models_json = root["models"];
    SceneGraph sg_data;
    for(int model_id = 0;model_id < models_json.size();++model_id)
    {
        Model model;
        Json::Value bodies_json = models_json[model_id]["bodies"];
        model.name = models_json[model_id]["name"].asString();
        model.bbox = readJsonVector(models_json[model_id]["boundingBox"]);
        for(int body_id = 0;body_id < bodies_json.size();++body_id)
        {
            Body body;
            body.name = bodies_json[body_id]["name"].asString();
            body.url = bodies_json[body_id]["url"].asString();
            body.collision_url = bodies_json[body_id]["collision_url"].asString();
            body.transform = readJsonMatrix(bodies_json[body_id]["transformation"]);
            model.bodies.push_back(body);
        }
        sg_data.models.push_back(model);
    }

    std::cout << "============ associateScene ===================" << std::endl;

    std::vector<bpp_actor::Actor> actors;
    actors.erase(actors.begin(), actors.end());
    actors.clear();

    for (size_t j = 0; j < sg_data.models.size(); ++j)
    {        
        bpp_actor::Actor actor;
        actor.uuid=sg_data.models[j].name;
        actor.semanticName=actor.uuid;
        actor.type = "static";
//        actor.header.seq = 0;
//        actor.header.stamp = ros::Time::now();
//        actor.header.frame_id = "base";
//        if(actor.uuid != "guedel" && actor.uuid != "p6ppallet")
//            actor.zone = "static";

        Eigen::Vector3d bbox = sg_data.models[j].bbox;
        actor.bbox.x = bbox(0);
        actor.bbox.y = bbox(1);
        actor.bbox.z = bbox(2);

        actor.modelPath=sg_data.models[j].url;

        actor.targetPoseVec.resize(sg_data.models[j].bodies.size());
        actor.desiredPoseVec.resize(actor.targetPoseVec.size());

        for (size_t k = 0; k < sg_data.models[j].bodies.size(); ++k)
        {

            std::string body_url = sg_data.models[j].bodies[k].url;
            std::string body_collision_url = sg_data.models[j].bodies[k].collision_url;

            std::cout << "Body URL " << body_url << std::endl;

            actor.body_urls.push_back(body_url);
            actor.body_collision_urls.push_back(body_collision_url);
            actor.bodies.push_back(sg_data.models[j].bodies[k].name);
//            actor.zone = "measurement_area";

            Eigen::Matrix4d tr = sg_data.models[j].bodies[k].transform;
            Eigen::Matrix3d R = tr.block(0,0,3,3);
            Eigen::Quaterniond q(R);

            actor.targetPoseVec[k].position.x=tr(0,3);
            actor.targetPoseVec[k].position.y=tr(1,3);
            actor.targetPoseVec[k].position.z=tr(2,3);
            actor.targetPoseVec[k].orientation.x=q.x();
            actor.targetPoseVec[k].orientation.y=q.y();
            actor.targetPoseVec[k].orientation.z=q.z();
            actor.targetPoseVec[k].orientation.w=q.w();
            actor.desiredPoseVec[k] = actor.targetPoseVec[k];
        }

        actors.push_back(actor);
    }

    return actors;
}

Json::Value getGeometryPoseJSON(geometry_msgs::Pose pose)
{
    Json::Value pose_val;
    pose_val["position"]["x"] = pose.position.x;
    pose_val["position"]["y"] = pose.position.y;
    pose_val["position"]["z"] = pose.position.z;
    pose_val["orientation"]["x"] = pose.orientation.x;
    pose_val["orientation"]["y"] = pose.orientation.y;
    pose_val["orientation"]["z"] = pose.orientation.z;
    pose_val["orientation"]["w"] = pose.orientation.w;
    return pose_val;
}

geometry_msgs::Pose getGeometryPose(Json::Value pose_val)
{
    geometry_msgs::Pose pose;
    pose.position.x = pose_val["position"]["x"].asDouble();
    pose.position.y = pose_val["position"]["y"].asDouble();
    pose.position.z = pose_val["position"]["z"].asDouble();
    pose.orientation.x = pose_val["orientation"]["x"].asDouble();
    pose.orientation.y = pose_val["orientation"]["y"].asDouble();
    pose.orientation.z = pose_val["orientation"]["z"].asDouble();
    pose.orientation.w = pose_val["orientation"]["w"].asDouble();
    return pose;
}

geometry_msgs::Vector3 getGeometryVector3(Json::Value vec3_val)
{
    geometry_msgs::Vector3 vec;
    vec.x = vec3_val["x"].asDouble();
    vec.y = vec3_val["y"].asDouble();
    vec.z = vec3_val["z"].asDouble();
    return vec;
}

Json::Value getGeometryVector3JSON(geometry_msgs::Vector3 vec3)
{
    Json::Value vec3_val;
    vec3_val["x"] = vec3.x;
    vec3_val["y"] = vec3.y;
    vec3_val["z"] = vec3.z;
    return vec3_val;
}

bool uuidAlreadyIncluded(std::string uuid, std::vector<bpp_actor::Actor> &actors){
    for(bpp_actor::Actor actor : actors){
        if(actor.uuid == uuid)
            return true;
    }
    return false;
}

bool SGJSONParser::loadActors(const std::string json_string, std::vector<bpp_actor::Actor> &actors)
{
    Json::Value box_list;
    Json::Reader reader;
    bool parsingSuccessful = reader.parse(json_string, box_list);
    if ( !parsingSuccessful )
    {
        // report to the user the failure and their locations in the document.
        std::cout  << "Failed to parse configuration: "<< reader.getFormattedErrorMessages();
        return false;
    }
    //TODO: return false if parsing fails
    return loadActors(box_list, actors);
}

bool SGJSONParser::saveActors(std::string &json_string, std::vector<bpp_actor::Actor> &actors)
{
    //TODO: return false if parsing fails
    Json::Value box_list;
    saveActors(box_list, actors);
    Json::StyledWriter writer;
    json_string = writer.write(box_list);
    return true;
}


bool SGJSONParser::loadBoxPlan(const std::string json_string, std::vector<bpp_msgs::BoxPlan> &boxplan_vec)
{
    Json::Value boxplan_list;
    Json::Reader reader;
    bool parsingSuccessful = reader.parse(json_string, boxplan_list);
    if ( !parsingSuccessful )
    {
        // report to the user the failure and their locations in the document.
        std::cout  << "Failed to parse configuration: "<< reader.getFormattedErrorMessages();
        return false;
    }
    return loadBoxPlan(boxplan_list, boxplan_vec);
}

bool SGJSONParser::saveBoxPlan(std::string &json_string, std::vector<bpp_msgs::BoxPlan> &boxplan_vec)
{
    //TODO: return false if parsing fails
    Json::Value boxplan_list;
    saveBoxPlan(boxplan_list, boxplan_vec);
    Json::StyledWriter writer;
    json_string = writer.write(boxplan_list);
    return true;
}

bool SGJSONParser::loadActors(Json::Value box_list, std::vector<bpp_actor::Actor> &actors)
{
    for(unsigned int box_id = 0; box_id < box_list.size();++box_id)
    {
        Json::Value box = box_list[box_id];
        bpp_actor::Actor actor;

        actor.uuid = box["uuid"].asString();
        //won't add boxes that are already added, based on their uuid
        if(uuidAlreadyIncluded(actor.uuid,actors)){
            continue;
        }
        actor.barcode = box["barcode"].asString();
        actor.storage_location = box["storage_location"].asString();
        actor.semanticName = box["semanticName"].asString();
        actor.zone = box["zone"].asString();
        actor.state = box["state"].asString();
        actor.type = box["type"].asString();
        Json::Value bodies = box["bodies"];
        for(unsigned int body_id = 0;body_id < bodies.size();++body_id)
            actor.bodies.push_back(bodies[body_id].asString());
        Json::Value body_urls = box["body_urls"];
        for(unsigned int body_id = 0;body_id < body_urls.size();++body_id)
            actor.body_urls.push_back(body_urls[body_id].asString());

        Json::Value body_texture_urls = box["body_texture_urls"];
        for(unsigned int texture_id = 0;texture_id < body_texture_urls.size();++texture_id)
            actor.body_texture_urls.push_back(body_texture_urls[texture_id].asString());

        Json::Value body_collision_urls = box["body_collision_urls"];
        for(unsigned int body_collision_id = 0;body_collision_id < body_texture_urls.size();++body_collision_id)
            actor.body_collision_urls.push_back(body_collision_urls[body_collision_id].asString());

        double x = box["bbox"]["x"].asDouble();
        double y = box["bbox"]["y"].asDouble();
        double z = box["bbox"]["z"].asDouble();
        actor.bbox.x = (( round(x * 100)))/ 100.0;
        actor.bbox.y = (( round(y * 100)))/ 100.0;
        actor.bbox.z = (( round(z * 100)))/ 100.0;
//        double x_2 = (( round(x * 10)))/ 10.0;
//        double y_2 = (( round(y * 10)))/ 10.0;
//        double z_2 = (( round(z * 10)))/ 10.0;
//        if(std::fabs(actor.bbox.x-x_2) < 0.012) {actor.bbox.x = x_2;}
//        if(std::fabs(actor.bbox.y-y_2) < 0.012) {actor.bbox.y = y_2;}
//        if(std::fabs(actor.bbox.z-z_2) < 0.012) {actor.bbox.z = z_2;}

        actor.weight = box["weight"].asDouble();
        actor.material = box["material"].asString();

        Json::Value labels = box["labels"];
        for(unsigned int i = 0;i < labels.size();++i)
            actor.labels.push_back(labels[i].asString());

        actor.fragile = box["fragile"].asBool();

        Json::Value targetPoseVec = box["targetPoseVec"];
        for(unsigned int i = 0;i < targetPoseVec.size();++i)
            actor.targetPoseVec.push_back(getGeometryPose(targetPoseVec[i]));

        Json::Value desiredPoseVec = box["desiredPoseVec"];
        for(unsigned int i = 0;i < desiredPoseVec.size();++i)
            actor.desiredPoseVec.push_back(getGeometryPose(desiredPoseVec[i]));
        if(desiredPoseVec.size() == 0)
        {
            actor.desiredPoseVec.resize(actor.targetPoseVec.size());
            actor.desiredPoseVec[0] = actor.targetPoseVec[0];
        }

        actor.graspPose = getGeometryPose(box["graspPose"]);
        actor.placePose = getGeometryPose(box["placePose"]);
        actor.robotTool = box["robotTool"].asString();
        actor.boxDirection = box["boxDirection"].asString();
        actor.associatedActor = box["associatedActor"].asString();
        actor.modelPath = box["modelPath"].asString();
        if(actor.bodies.size() == 0 || actor.body_urls.size() == 0)
        {
            std::string model_fname = generateWRLFile(actor);
            std::stringstream ss_url;
            ss_url << "http://www.ac2.sg/blob_store/" << actor.uuid << ".wrl";
            actor.modelPath = ss_url.str();
            actor.bodies.push_back(actor.uuid);
            actor.body_urls.push_back(ss_url.str());
            if(!blobhelper::BlobStoreHelper::addBlob(std::string(model_fname.c_str()), std::string(ss_url.str().c_str())))
                std::cout << "blob store upload failed" << std::endl;
        }
        actor.isArticulated = box["isArticulated"].asBool();
        actor.kinematicModelPath = box["kinematicModelPath"].asString();
        //TODO: save fitting point information??

        Json::Value face_urls = box["facesPath"];
        for(unsigned int i = 0;i < face_urls.size();++i)
            actor.facesPath.push_back(face_urls[i].asString());

        /*****/
        Json::Value rectLabelVecs_json = box["rectLabelVecs"];
        for(unsigned int i = 0; i<rectLabelVecs_json.size(); i++){
            bpp_actor::RectLabelVec rectLabelVec;
            Json::Value rectLabelVec_json = rectLabelVecs_json[i];
            for(unsigned int j=0; j<rectLabelVec_json.size(); j++){
                Json::Value rectLabel_json = rectLabelVec_json[j];
                bpp_actor::RectLabel rectLabel;
                rectLabel.x = rectLabel_json["x"].asInt();
                rectLabel.y = rectLabel_json["y"].asInt();
                rectLabel.w = rectLabel_json["w"].asInt();
                rectLabel.h = rectLabel_json["h"].asInt();
                rectLabel.label = rectLabel_json["label"].asInt();
                rectLabelVec.push_back(rectLabel);
            }
            actor.rectLabelVecs.push_back(rectLabelVec);
        }
        /*****/

        actor.featurePointsPath = box["featurePointsPath"].asString();
        actor.measureDebugPath = box["measureDebugPath"].asString();
        actor.objectPointsPath = box["objectPointsPath"].asString();
        actor.meshPath = box["meshPath"].asString();
        actor.thumbnailPath = box["thumbnailPath"].asString();
        actor.markerId = box["markerId"].asString();
        actor.cog = getGeometryVector3(box["cog"]);

        actors.push_back(actor);
    }
    return true;
}

bool SGJSONParser::saveActors(Json::Value &box_list, std::vector<bpp_actor::Actor> &actors)
{
    for(bpp_actor::Actor actor: actors)
    {
        Json::Value actor_json;
        actor_json["uuid"] = actor.uuid;
        actor_json["barcode"] = actor.barcode;
        actor_json["storage_location"] = actor.storage_location;
        actor_json["semanticName"] = actor.semanticName;
        actor_json["zone"] = actor.zone;
        actor_json["state"] = actor.state;
        actor_json["type"] = actor.type;
        for(std::string body: actor.bodies)
            actor_json["bodies"].append(body);
        for(std::string body_url: actor.body_urls)
            actor_json["body_urls"].append(body_url);
        for(std::string body_texture: actor.body_texture_urls)
            actor_json["body_texture_urls"].append(body_texture);
        for(std::string body_collision_texture: actor.body_collision_urls)
            actor_json["body_collision_textures"].append(body_collision_texture);
        actor_json["bbox"]["x"] = actor.bbox.x;    // make sure you have set your locale correctly:    echo 'export LC_NUMERIC="C"' >>~/.bashrc
        actor_json["bbox"]["y"] = actor.bbox.y;    // otherwise double will be formated with , instead of . and will mess up the json file
        actor_json["bbox"]["z"] = actor.bbox.z;

        actor_json["weight"] = actor.weight;
        actor_json["material"] = actor.material;
        for(std::string label: actor.labels)
            actor_json["labels"].append(label);
        actor_json["fragile"] = actor.fragile;
        for(geometry_msgs::Pose pose: actor.targetPoseVec)
        {
            actor_json["targetPoseVec"].append(getGeometryPoseJSON(pose));
        }
        for(geometry_msgs::Pose pose: actor.desiredPoseVec)
        {
            actor_json["desiredPoseVec"].append(getGeometryPoseJSON(pose));
        }
        actor_json["graspPose"] = getGeometryPoseJSON(actor.graspPose);
        actor_json["placePose"] = getGeometryPoseJSON(actor.placePose);
        actor_json["robotTool"] = actor.robotTool;
        actor_json["boxDirection"] = actor.boxDirection;
        actor_json["associatedActor"] = actor.associatedActor;
        actor_json["modelPath"] = actor.modelPath;
        actor_json["isArticulated"] = actor.isArticulated;
        actor_json["kinematicModelPath"] = actor.kinematicModelPath;
        //TODO: save fitting point information??
        for(std::string face_url: actor.facesPath)
            actor_json["facesPath"].append(face_url);

        /*****/
        for(bpp_actor::RectLabelVec rectLabelVec : actor.rectLabelVecs){
            Json::Value rectLabelVec_json;
            for(bpp_actor::RectLabel rectLabel : rectLabelVec){
                Json::Value rectLabel_json;
                rectLabel_json["x"] = rectLabel.x;
                rectLabel_json["y"] = rectLabel.y;
                rectLabel_json["w"] = rectLabel.w;
                rectLabel_json["h"] = rectLabel.h;
                rectLabel_json["label"] = rectLabel.label;
                rectLabelVec_json.append(rectLabel_json);
            }
            actor_json["rectLabelVecs"].append(rectLabelVec_json);
        }
        /*****/

        actor_json["featurePointsPath"] = actor.featurePointsPath;
        actor_json["measureDebugPath"] = actor.measureDebugPath;
        actor_json["objectPointsPath"] = actor.objectPointsPath;
        actor_json["meshPath"] = actor.meshPath;
        actor_json["thumbnailPath"] = actor.thumbnailPath;
        actor_json["markerId"] = actor.markerId;
        actor_json["cog"] = getGeometryVector3JSON(actor.cog);

        box_list.append(actor_json);
    }
    return true;
}


bool SGJSONParser::saveBoxPlan(Json::Value &boxplan_list, std::vector<bpp_msgs::BoxPlan> &boxplan_vec)
{
    for(bpp_msgs::BoxPlan boxplan: boxplan_vec)
    {
        Json::Value boxplan_json;
        boxplan_json["box_uuid"] = boxplan.box_uuid;
        boxplan_json["box_pose_in_pallet"] = getGeometryPoseJSON(boxplan.box_pose_in_pallet);
        boxplan_json["is_rotated"] = boxplan.is_rotated;
        boxplan_json["bbox"] = getGeometryVector3JSON(boxplan.bbox);
        boxplan_json["box_mass"] = boxplan.box_mass;
        boxplan_json["box_state"] = boxplan.box_state;
        boxplan_json["material"] = boxplan.material;

        for(std::string label: boxplan.labels)
            boxplan_json["labels"].append(label);

        boxplan_list.append(boxplan_json);
    }
    return true;
}

bool SGJSONParser::loadBoxPlan(Json::Value boxplan_list, std::vector<bpp_msgs::BoxPlan> &boxplan_vec)
{
    for(unsigned int boxplan_id = 0; boxplan_id < boxplan_list.size();++boxplan_id)
    {
        Json::Value boxplan_json = boxplan_list[boxplan_id];
        bpp_msgs::BoxPlan boxplan;

        boxplan.box_uuid = boxplan_json["box_uuid"].asString();
        boxplan.box_pose_in_pallet = getGeometryPose(boxplan_json["box_pose_in_pallet"]);
        boxplan.is_rotated = boxplan_json["is_rotated"].asBool();
        boxplan.bbox = getGeometryVector3(boxplan_json["bbox"]);
        boxplan.box_mass = boxplan_json["box_mass"].asDouble();
        boxplan.box_state = boxplan_json["box_state"].asString();
        boxplan.material = boxplan_json["material"].asString();

        Json::Value labels = boxplan_json["labels"];
        for(unsigned int i = 0;i < labels.size();++i)
            boxplan.labels.push_back(labels[i].asString());

        boxplan_vec.push_back(boxplan);
    }
    return true;
}

std::string SGJSONParser::generateWRLFile(bpp_actor::Actor &box)
{
    int max_value = 150;
    float value = 255.0;
    float colorR, colorG, colorB;
    if(box.material == "styrofoam")
    {
        //red
        colorR = (float) (255 - rand() % 55) / value;  //200-255
        colorG = (float) (rand() % max_value) / value;
        colorB = (float) (rand() % max_value) / value;
    }
    else if(box.material == "wooden")
    {
        //green
        colorR = (float) (rand() % max_value) / value;
        colorG = (float) (255 - rand() % 55) / value;  //200-255
        colorB = (float) (rand() % max_value) / value;
    }
    else //carton
    {
        //blue
        colorR = (float) (rand() % max_value) / value;
        colorG = (float) (rand() % max_value) / value;
        colorB = (float) (255 - rand() % 55) / value;  //200-255
    }

    std::stringstream fname;
    fname << "/tmp/" << box.uuid << ".wrl";
    std::ofstream writer;
    writer.open (fname.str(), std::ios::out);
    writer << "#VRML V2.0 utf8 \n Transform { \n \t children [ \n \t \t DEF box Transform { \n \t \t \t children [ \n \t \t \t \t DEF body1 Transform { \n \t \t \t \t \t";
    writer << "translation 0 0 0" << "\n \t \t \t \t \t";
    writer << "children [ \n \t \t \t \t \t \t Shape { \n";
    writer << "\t \t \t \t \t \t \t appearance Appearance { \n \t \t \t \t \t \t \t \t material Material { \n \t \t \t \t \t \t \t \t \t diffuseColor " <<colorR << " " << colorG << " " << colorB<<" \n \t \t \t \t \t \t \t \t } \n \t \t \t \t \t \t \t } \n";
    writer << "\t \t \t \t \t \t \t geometry Box { \n \t \t \t \t \t \t \t \t size " << box.bbox.x << " " << box.bbox.y << " " << box.bbox.z << "\n \t \t \t \t \t \t \t } \n ";

    writer << "\t \t \t \t \t \t } \n \t \t \t \t \t ] \n \t \t \t \t } \n \t \t \t ] \n \t \t } \n \t ] \n } \n";
    writer.close();
    return fname.str();
}
