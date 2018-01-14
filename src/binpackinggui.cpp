#include "binpackinggui.h"
#include <QStringListModel>
#include <QMovie>
#include <QFileDialog>
#include <gui_lib/flickcharm.h>
#include "mainwindow.h"
#include <qprogressbar.h>
#include <bullet/Actor.h>

BinPackingGUI::BinPackingGUI(QWidget *parent) :
    SceneRenderer3D(parent),
    ui(new Ui::BinPackingGUI),
    msgBox(this),
    saveSceneAction(new QAction(this)),
    movie_loading_(new QMovie(":/images/load2.gif")),
    bpp_opt_goal_("none")
{
    this->instance_name_ = std::string("bpp-");
    setPlanningFinished(false);
    this->timer_id_ = startTimer(1);
    ui->setupUi(this);

//    this->ui->imageHolder->addWidget(this->viewer_->getWidget(), 0, 0);  not working so far
    this->viewer_->getWidget()->setParent(this->ui->imageHolder);
    this->viewer_->getWidget()->lower();
//    this->ui->defaultviewBtn->show();
    this->setWindowIconText("Bin Packing Planner");
    this->setWindowTitle("Bin Packing Planner");

    msgBox.setWindowTitle("Bin Packing Planner");
    msgBox.setText("Planning in progress, please wait ...");


    this->saveSceneAction->setShortcut(QKeySequence("Ctrl+Return"));
    QObject::connect(this->saveSceneAction, SIGNAL(triggered()), this, SLOT(saveScene()));
    this->addAction(this->saveSceneAction);

    //configure list to show text+icon
    this->ui->listWidget->setIconSize(QSize(100,100));
    this->ui->listWidget->setMovement(QListWidget::Static);
    this->ui->listWidget->setViewMode(QListWidget::IconMode);//deactivate this to look more like a list
    this->ui->listWidget->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);

    //set up scrolling of list
    FlickCharm *charm = new FlickCharm();
    charm->activateOn(this->ui->listWidget);

    //enable toggle buttons
//    this->ui->defaultviewBtn->setCheckable(true);
    show_desired_pose_ = false;

    this->ui->nextBtn->setEnabled(false);
    on_radioButton_default_clicked(true);
    this->ui->radioButton_default->setChecked(true);

    this->ui->sortall_value->setText("0");
    this->ui->sortplanned_value->setText("0");
    this->ui->sortunplanned_value->setText("0");
    importedBoxesCount = 0;

    this->ui->progress->setText(" ");
    //    this->ui->progressBar->setTextVisible(true);
    //    this->ui->progressBar->setFormat("");
    //    this->ui->progressBar->setValue(0);

    this->ui->checkbox_carton->setChecked(true);
    this->ui->checkbox_dangerous->setChecked(true);
    this->ui->checkbox_fragile->setChecked(true);
    this->ui->checkbox_others->setChecked(true);
    this->ui->checkbox_plastic->setChecked(true);
    this->ui->checkbox_styrofoam->setChecked(true);
    this->ui->checkbox_wooden->setChecked(true);
//    this->ui->textureviewBtn->setChecked(false);

    viewer_->setTransparencyType(SoGLRenderAction::TransparencyType::SORTED_OBJECT_SORTED_TRIANGLE_BLEND);
    this->switchCameraView(0);

    this->ui->stackedWidget->setCurrentIndex(0); // always start with the first page of the stacked widget
    this->ui->volume_label->setText("volume (m<sup>3</sup>)");

    std::cout << __LINE__ << std::endl;


    std::string mem;
    if(!blobhelper::BlobStoreHelper::getBlob(std::string("file:///home/nair/data/blob_store/ac2.json"), mem)){
        std::cout << "ERROR: Get Blob Failed!" << std::endl;
        throw;
    }

    std::cout << mem << std::endl;

    std::string sg_json(mem.c_str());

    std::cout << __LINE__ << std::endl;

    std::vector<bpp_actor::Actor > actors_sg=parser.loadScene(sg_json);
    for(auto actor:actors_sg){

        ws_.add_actor(actor);
        ObjectModel model=createModel(actor);
        addObjectModel(model);

        if(actor.uuid=="p6ppallet"){
            std::cout << "p6ppallet found -----------> seetin transform wrt base" << std::endl;
            tf::poseMsgToEigen(actor.targetPoseVec[0], pallet_tr_to_world);

        }

    }
}

BinPackingGUI::~BinPackingGUI()
{
    delete ui;
}

QMatrix4x4 BinPackingGUI::poseToQMatrix4x4(geometry_msgs::Pose pose)
{
    QMatrix4x4 qm;

    Eigen::Affine3d eigen_tr;
    tf::poseMsgToEigen(pose, eigen_tr);

    for(size_t i=0;i<4;++i){
        for(size_t j=0;j<4;++j){
            qm(i,j)=eigen_tr(i,j);
        }
    }
    return qm;
}

void BinPackingGUI::updateSceneModels()
{
    std::vector<bpp_actor::Actor > actors;
    bpp_actor::QueryActor query;
    query.all=true;
    ws_.get_actor_callback(query, actors);

    for(auto actor:actors){

        ObjectModel model=createModel(actor);
        updateObjectModel(model);
    }
}

void BinPackingGUI::updateScenePlanningModels()
{
    std::vector<bpp_actor::Actor > actors;
    bpp_actor::QueryActor query;
    query.type="box";
    ws_.get_actor_callback(query, actors);

    for(auto actor:actors){

        ObjectModel model=createModel(actor);
        updateObjectModel(model);
    }
}


ObjectModel BinPackingGUI::createModel(const bpp_actor::Actor &actor)
{
    ObjectModel model;
    model.object_name=actor.uuid.c_str();
    for(size_t i=0;i<actor.bodies.size();++i)
    {
        ObjectBody body;
        body.body_name=actor.bodies[i].c_str();
        body.mesh_url=actor.body_urls[i].c_str();
        body.tr=poseToQMatrix4x4(actor.targetPoseVec[i]);
        body.actor.type = actor.type;
        body.actor.labels=actor.labels;
        body.actor.bbox.setX(actor.bbox.x);
        body.actor.bbox.setY(actor.bbox.y);
        body.actor.bbox.setZ(actor.bbox.z);
        body.actor.material=actor.material;
        body.actor.uuid=actor.uuid;
        model.bodies.push_back(body);
    }

    if(actor.body_texture_urls.size() != 0)
    {
//        std::cout << actor.body_texture_urls[0] << std::endl;
        std::stringstream ss_tex_url;
        ss_tex_url << "file:///home/nair/data/blob_store/" << actor.uuid << ".png";
        saveFileLocally(actor.body_texture_urls[0], ss_tex_url.str(), "", true);
    }

    return model;
}

bool BinPackingGUI::getShowDesiredPose()
{
    bool val;
    mutex_show_desired_pose_.lock();
    val = show_desired_pose_;
    mutex_show_desired_pose_.unlock();
    return val;
}

void BinPackingGUI::setShowDesiredPose(bool sdp)
{
    mutex_show_desired_pose_.lock();
    show_desired_pose_ = sdp;
    mutex_show_desired_pose_.unlock();
}

bool BinPackingGUI::getPlanningFinished()
{
    bool val;
    mutex_planning_finished_.lock();
    val = planning_finished_;
    mutex_planning_finished_.unlock();
    return val;
}

void BinPackingGUI::setPlanningFinished(bool plan_flag)
{
    mutex_planning_finished_.lock();
    planning_finished_ = plan_flag;
    mutex_planning_finished_.unlock();
}

bool BinPackingGUI::getPlanningRequested()
{
    bool val;
    mutex_planning_request_.lock();
    val = planning_request_;
    mutex_planning_request_.unlock();
    return val;
}

void BinPackingGUI::setPlanningRequested(bool plan_flag)
{
    mutex_planning_request_.lock();
    planning_request_ = plan_flag;
    mutex_planning_request_.unlock();
}

bpp_actor::ActorVec BinPackingGUI::getPlanActors()
{
    bpp_actor::ActorVec actors;
    mutex_plan_actors_.lock();
    actors = plan_actors_;
    mutex_plan_actors_.unlock();
    return actors;
}

void BinPackingGUI::setPlanActors(bpp_actor::ActorVec actors)
{
    mutex_plan_actors_.lock();
    plan_actors_ = actors;
    mutex_plan_actors_.unlock();
}

void BinPackingGUI::setSelectedActor(bpp_actor::Actor actor)
{
    std::cout << "called: " << actor.uuid << std::endl;
    ui->boxID_value->setText(QString(actor.uuid.c_str()));
    ui->boxweight_value->setText(QString::number(actor.weight, 'f', 2));
    ui->length_value->setText(QString::number(actor.bbox.x, 'f', 2));
    ui->width_value->setText(QString::number(actor.bbox.y, 'f', 2));
    ui->height_value->setText(QString::number(actor.bbox.z, 'f', 2));
    ui->material_value->setText(QString(actor.material.c_str()));
    ui->storage_value->setText(QString(actor.storage_location.c_str()));
    ui->tool_value->setText(QString(actor.robotTool.c_str()));
    ui->support_value->setText(QString::number(actor.supportBoxes.size()));
    this->supporting_boxes.clear();
    for(bpp_actor::SupportBox sbox : actor.supportBoxes)
    {
        this->supporting_boxes.push_back(sbox.boxID);
    }

    std::cout << "Box " << actor.uuid << " has " << actor.labels.size() << " labels\n";
    if(actor.labels.size()>0)
    {
        QString labels = QString(actor.labels[0].c_str());
        for(int i=1; i < actor.labels.size(); i++)
            labels += ", " + QString(actor.labels[i].c_str());
        ui->label_1->setText(labels); //removed
    }
    else
        ui->label_1->setText(QString("none")); //removed

    QString size = QString::number(actor.bbox.x) + ", "+ QString::number(actor.bbox.y) + ", " + QString::number(actor.bbox.z);
//    ui->lineEdit_bbox->setText(size); //removed
}

EvalBppResults BinPackingGUI::getBppResults()
{
    EvalBppResults results;
    mutex_bpp_results_.lock();
    results = bpp_results_;
    mutex_bpp_results_.unlock();
    return results;
}

void BinPackingGUI::setBppResults(EvalBppResults bpp_results)
{
    mutex_bpp_results_.lock();
    bpp_results_ = bpp_results;
    mutex_bpp_results_.unlock();
}

void BinPackingGUI::closeEvent(QCloseEvent *event)
{
    std::cout << "Closing bpp gui" << std::endl;

    killTimer(this->timer_id_);
}

void BinPackingGUI::timerEvent(QTimerEvent* timerEvent)
{
    if(getPlanningFinished())
    {
        setPlanningFinished(false);
        planReceived();       
        this->ui->nextBtn->setEnabled(true);
    }
//    std::cout << selected_actor << std::endl;

//    std::pair<bool, std::string> objectSelection = getSelectedObject();                                                                   //removed
//    if(objectSelection.first && objectSelection.second != "" && objectSelection.second != ui->groupBox->title().toStdString())
//    {
//        ui->groupBox->setTitle(QString(objectSelection.second.c_str()));
//        emit requestActorFromUUID(objectSelection.second);
//        QList<QListWidgetItem*> itemList = ui->listWidget->findItems(QString(objectSelection.second.c_str()), Qt::MatchExactly);
//        ui->listWidget->setCurrentItem(*itemList.begin());
//        on_listWidget_itemSelectionChanged();
//    }
}

void BinPackingGUI::on_planBtn_clicked()
{
    ui->progress->setText("    ");
    ui->progress->setMovie(movie_loading_);
    movie_loading_->start();
//    ui->progressBar->setFormat("Planning......");
//    ui->progressBar->setValue(0);
    QApplication::processEvents(); //process all events now!

    std::cout << "Plan button clicked" << std::endl;
    emit requestBinPackingPlan(bpp_opt_goal_);
}

void BinPackingGUI::planReceived()
{
    ui->planBtn->setText("Replan");
    ui->progress->setText("  ");
    ui->progress->setPixmap(QPixmap(":/images/bpp.png"));
//    ui->progressBar->setValue(100);

    // process all events now!
    QApplication::processEvents();

    int dangerousCnt = 0, fragileCnt = 0;

    // show loaded boxes with simulated standard pics, this is should be real one for real boxes data
    bpp_actor::ActorVec actorVec = getPlanActors();


    std::cout << "Updating box ----------------------------------------> "  << std::endl;

    //update in gui
    for(auto actor:actorVec){
        ObjectModel model=createModel(actor);
        std::cout << "Updating box ----------------------------------------> " << model.object_name.toStdString() << std::endl;

        updateObjectModel(model);
        updateObjectModelPlanning(model);
    }


   	this->ui->listWidget->clear();
    for(bpp_actor::Actor &actor: actorVec)
    {
        std::cout << "Adding box viz ---------> " << actor.uuid << std::endl;
//        std::string thumbnailPath = getThumbnailPath("file:///home/nair/data/blob_store/", actor.uuid);
        std::string thumbnailPath = getThumbnailPath("file:///home/nair/data/blob_store/", actor.uuid);

        if(saveFileLocally(actor.thumbnailPath, thumbnailPath) && actor.thumbnailPath.size() != 0)
            this->ui->listWidget->addItem(new QListWidgetItem(QIcon(QString::fromStdString(thumbnailPath.c_str())),actor.uuid.c_str()));
        else
        {
            if(actor.material == "wooden" || actor.material == "Wooden")
                this->ui->listWidget->addItem(new QListWidgetItem(QIcon(QPixmap(":/images/wooden.png")),actor.uuid.c_str()));
            else if(actor.material == "carton" || actor.material == "Carton")
                this->ui->listWidget->addItem(new QListWidgetItem(QIcon(QPixmap(":/images/carton.png")),actor.uuid.c_str()));
            else if(actor.material == "styrofoam" || actor.material == "Styrofoam")
                this->ui->listWidget->addItem(new QListWidgetItem(QIcon(QPixmap(":/images/styrofoam.png")),actor.uuid.c_str()));
            else if(actor.material == "plastic" || actor.material == "Plastic")
                this->ui->listWidget->addItem(new QListWidgetItem(QIcon(QPixmap(":/images/plastic.png")),actor.uuid.c_str()));
            else if(actor.material == "drum" || actor.material == "Drum")
                this->ui->listWidget->addItem(new QListWidgetItem(QIcon(QPixmap(":/images/plastic.png")),actor.uuid.c_str()));
            else
                this->ui->listWidget->addItem(new QListWidgetItem(QIcon(QPixmap(":/images/box_icon_big.png")),actor.uuid.c_str()));
        }

        for(std::string label : actor.labels)
        {
            if(label == "fragile")
                fragileCnt++;
            if(label == "flammable" || label == "flammableLiquid" || label == "dangerous")
                dangerousCnt++;
        }
    }
    for(size_t id = 0; id < this->ui->listWidget->count(); id++)
    {
            QListWidgetItem *itm = this->ui->listWidget->item(id);
            itm->setTextColor(Qt::white);
    }

    // after bin planning, call the evaluation automatically
    this->on_evalButton_clicked();

    this->ui->fragile_value->setText(QString::number(fragileCnt));
    this->ui->dangerous_value->setText(QString::number(dangerousCnt));

    this->ui->nextBtn->setEnabled(true);

    //show planned pose
    on_plannedviewBtn_clicked(true);
}

//cai: didn't inial show_desired_pose_, so sometime it = 61, has a bug.
void BinPackingGUI::on_showPlanedPose_clicked(bool checked)
{
//    setShowDesiredPose(!show_desired_pose_);
    setShowDesiredPose(checked);
    std::cout << "show_desired pose = " << show_desired_pose_ << std::endl;
}

void BinPackingGUI::on_MASimulationButton_clicked()
{
    emit requestMALocation();
}

void BinPackingGUI::on_resetviewBtn_clicked()
{
    viewer_->viewAll();
    viewer_->setTransparencyType(SoGLRenderAction::TransparencyType::SORTED_OBJECT_SORTED_TRIANGLE_BLEND);
//    backupCurrentWorldStateSignal(std::string("latest"));
}

void BinPackingGUI::on_evalButton_clicked()
{
    std::cout << "start evaluation bpp ........................\n";
    emit evaluateBinPackingPlan();

    EvalBppResults result = getBppResults();
    ui->volume_value->setText( QString::number(result.bpp_used_space_, 'f', 2) + " ("+QString::number((result.bpp_used_space_/(3*3*2.2))*100, 'f', 2)+"%)");
    ui->weight_value->setText( QString::number(result.bpp_packed_mass_, 'f', 2) );

    //cut for 0.000
    double x = ((int)( floor((result.bpp_com_.x()+1e-3) * 1000)))/ 1000.0;
    double y = ((int)( floor((result.bpp_com_.y()+1e-3) * 1000)))/ 1000.0;
    double z = ((int)( floor((result.bpp_com_.z()+1e-3) * 1000)))/ 1000.0;
    QString com = QString::number(x, 'f', 2) + ",  "+ QString::number(y, 'f', 2) + ",  " + QString::number(z, 'f', 2);
//    ui->COM_value->setText(com);

    // set the bin com for the 3D visualization
    com_x = (float)result.bpp_com_.x();
    com_y = (float)result.bpp_com_.y();
    com_z = (float)result.bpp_com_.z();

    this->ui->quantity_value->setText(QString::number(this->ui->listWidget->count()));

    this->ui->sortall_value->setText(QString::number(importedBoxesCount));
    this->ui->sortplanned_value->setText(QString::number(this->ui->listWidget->count()));
    this->ui->sortunplanned_value->setText(QString::number(importedBoxesCount - this->ui->listWidget->count()));
}

void BinPackingGUI::on_importBtn_clicked()
{
    this->ui->listWidget->clear();
    bpp_actor::ActorVec actorVec;
    loadBoxesJSON(actorVec);

    for(bpp_actor::Actor &actor: actorVec)
    {
//        std::string thumbnailPath = getThumbnailPath("file:///home/nair/data/blob_store/", a/ctor.uuid);
        std::string thumbnailPath = getThumbnailPath("/home/nair/data/blob_store/", actor.uuid);

        std::cout << "thumbnailPath +++++++++++++++++++++++++++++++++++++ " << thumbnailPath << std::endl;

        if(saveFileLocally(actor.thumbnailPath, thumbnailPath))
            this->ui->listWidget->addItem(new QListWidgetItem(QIcon(QString::fromStdString(thumbnailPath.c_str())),actor.uuid.c_str()));
        else
        {
            if(actor.material == "wooden" || actor.material == "Wooden")
                this->ui->listWidget->addItem(new QListWidgetItem(QIcon(QPixmap(":/images/wooden.png")),actor.uuid.c_str()));
            else if(actor.material == "carton" || actor.material == "Carton")
                this->ui->listWidget->addItem(new QListWidgetItem(QIcon(QPixmap(":/images/carton.png")),actor.uuid.c_str()));
            else if(actor.material == "styrofoam" || actor.material == "Styrofoam")
                this->ui->listWidget->addItem(new QListWidgetItem(QIcon(QPixmap(":/images/styrofoam.png")),actor.uuid.c_str()));
            else if(actor.material == "plastic" || actor.material == "Plastic")
                this->ui->listWidget->addItem(new QListWidgetItem(QIcon(QPixmap(":/images/plastic.png")),actor.uuid.c_str()));
            else if(actor.material == "drum" || actor.material == "Drum")
                this->ui->listWidget->addItem(new QListWidgetItem(QIcon(QPixmap(":/images/plastic.png")),actor.uuid.c_str()));
            else
                this->ui->listWidget->addItem(new QListWidgetItem(QIcon(QPixmap(":/images/box_icon_big.png")),actor.uuid.c_str()));
        }
    }
    for(size_t id = 0; id < this->ui->listWidget->count(); id++)
    {
            QListWidgetItem *itm = this->ui->listWidget->item(id);
            itm->setTextColor(Qt::white);
    }

    this->ui->saveButton->setEnabled(true);
    this->ui->MASimulationButton->setEnabled(true);   //removed

    //emit backupCurrentWorldStateSignal(std::string("boxes_loaded"));

    importedBoxesCount = this->ui->listWidget->count();
    this->ui->sortall_value->setText(QString::number(importedBoxesCount));
    this->ui->sortplanned_value->setText("0");
    this->ui->sortunplanned_value->setText(QString::number(importedBoxesCount));
}

void BinPackingGUI::saveBoxesJSON(bpp_actor::ActorVec &actors)
{
    throw;
//    QString dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"),"/home",QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks | QFileDialog::DontUseNativeDialog);
//    std::string folderPath = dir.toStdString() + "/";

//    if(!wsgtools::WSGClientHelper::saveActorsToJSONFolder(folderPath, actors)){
//        cerr << "Failed to save to JSON Folder" << endl;
//    }
}

void BinPackingGUI::loadBoxesJSON(bpp_actor::ActorVec &actors)
{
    QString dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"),"/home",QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks | QFileDialog::DontUseNativeDialog);
    std::string folderPath = dir.toStdString() + "/";

    if(!loadActorsFromJSONFolder(folderPath, actors)){
        cerr << "Failed to load from JSON Folder" << endl;
    }

    for(auto actor:actors){
        ws_.add_actor(actor);
        ObjectModel model=createModel(actor);
        addObjectModel(model);
    }
}

bool BinPackingGUI::loadActorsFromJSONFolder(std::string folderPath, bpp_actor::ActorVec &actors)
{
    // Load Json String
    std::ifstream jsonFile(folderPath + "boxes.json");
    if(!jsonFile.good()){
        cerr << "JSON File not found" << endl;
        return false;
    }
    std::stringstream jsonBuffer;
    jsonBuffer << jsonFile.rdbuf();
    std::string jsonString = jsonBuffer.str();
    jsonFile.close();

    // Convert to actor
    std::vector<bpp_actor::Actor> newActors;
    if(!parser.loadActors(jsonString, newActors)){
        cerr << "Failed to parse JSON" << endl;
        return false;
    }

    // Iterate
    for(bpp_actor::Actor actor : newActors){

        // Faces
        int faceID = 0;
        for(std::string faceURLPath : actor.facesPath){
            std::string path = getFacePath(folderPath, actor.uuid, std::to_string(faceID));
            if(fileExists(path)) saveFileBlobStore(getFaceURL(actor.uuid, std::to_string(faceID)), path);
            faceID++;
        }

        // Features
        if(actor.featurePointsPath.size()){
            std::string path = getFeaturePointsPath(folderPath, actor.uuid);
            if(fileExists(path)) saveFileBlobStore(getFeaturePointsURL(actor.uuid), path);
        }

        // ObjectPoints
        if(actor.objectPointsPath.size()){
            std::string path = getObjectPointsPath(folderPath, actor.uuid);
            if(fileExists(path)) saveFileBlobStore(getObjectPointsURL(actor.uuid), path);
        }


        // Measure Debug
        if(actor.measureDebugPath.size()){
            std::string path = getMeasureDebugPath(folderPath, actor.uuid);
            if(fileExists(path)) saveFileBlobStore(getMeasureDebugURL(actor.uuid), path);
        }

        // Mesh
        if(actor.meshPath.size()){
            std::string path = getMeshPathZipped(folderPath, actor.uuid);
            if(fileExists(path)) saveFileBlobStore(getMeshURLZipped(actor.uuid), path);
        }

        // Thumbnail
        if(actor.thumbnailPath.size()){
            std::string path = getThumbnailPath(folderPath, actor.uuid);
            if(fileExists(path)) saveFileBlobStore(getThumbnailURL(actor.uuid), path);
        }

        // WRL
        for(size_t body_id = 0;body_id < actor.bodies.size();++body_id){
            std::string path = getWRLPath(folderPath, actor.uuid, body_id);
            if(fileExists(path)) saveFileBlobStore(actor.body_urls[body_id], path);
        }

        // Texture Model
        if(actor.body_urls.size()){
            std::string path = folderPath + actor.uuid + ".wrl";
            std::string urlPath = "file:///home/nair/data/blob_store/" + actor.uuid + ".wrl";
            if(fileExists(path)){
                saveFileBlobStore(urlPath, path);
            }
        }
        if(actor.body_texture_urls.size()){
            std::string path = folderPath + actor.uuid + ".png";
            std::string urlPath = "file:///home/nair/data/blob_store/" + actor.uuid + ".png";
            if(fileExists(path)){
                saveFileBlobStore(urlPath, path);
            }
        }


        // WSG
        bpp_actor::Actor testActor;
        if(!ws_.uuidExists(actor.uuid))
            ws_.add_actor(actor);
        else
            ws_.update_actor(actor);

    }

    actors = newActors;
    return true;
}

void BinPackingGUI::on_listWidget_itemSelectionChanged()
{
    this->ui->stackedWidget->setCurrentIndex(1);
    QListWidgetItem *item = this->ui->listWidget->currentItem();
    QString itemName = item->data(Qt::DisplayRole).toString();

    setSelectedObject(true, itemName.toStdString());

    setSelectedActor(ws_.get_actor_from_uuid(itemName.toStdString())[0]);
//    ui->groupBox->setTitle(itemName);                     //removed
//    emit requestActorFromUUID(itemName.toStdString());
}


void BinPackingGUI::on_checkbox_wooden_clicked(bool checked)
{
    mutex_wooden_.lock();
    if(checked)
        transparency_wooden = false;
    else
        transparency_wooden = true;
    mutex_wooden_.unlock();
    updateScenePlanningModels();
}

void BinPackingGUI::on_checkbox_styrofoam_clicked(bool checked)
{
    mutex_styrofoam_.lock();
    if(checked)
        transparency_styrofoam = false;
    else
        transparency_styrofoam = true;
    mutex_styrofoam_.unlock();
    updateScenePlanningModels();

}

void BinPackingGUI::on_checkbox_carton_clicked(bool checked)
{
    mutex_carton_.lock();
    if(checked)
        transparency_carton = false;
    else
        transparency_carton = true;
    mutex_carton_.unlock();
    updateScenePlanningModels();

}

void BinPackingGUI::on_checkbox_plastic_clicked(bool checked)
{
    mutex_plastic_.lock();
    if(checked)
        transparency_plastic = false;
    else
        transparency_plastic = true;
    mutex_plastic_.unlock();
    updateScenePlanningModels();

}

void BinPackingGUI::on_checkbox_fragile_clicked(bool checked)
{
    mutex_fragile_.lock();
    if(checked)
        view_fragile = true;
    else
        view_fragile = false;
    mutex_fragile_.unlock();
    updateScenePlanningModels();

}

void BinPackingGUI::on_checkbox_dangerous_clicked(bool checked)
{
    mutex_dangerous_.lock();
    if(checked)
        view_dangerous = true;
    else
        view_dangerous = false;
    mutex_dangerous_.unlock();
    updateScenePlanningModels();

}

void BinPackingGUI::on_checkbox_others_clicked(bool checked)
{
    mutex_others_.lock();
    if(checked)
        view_others = true;
    else
        view_others = false;
    mutex_others_.unlock();
    updateScenePlanningModels();

}

void BinPackingGUI::on_clearBtn_clicked()
{
    on_checkbox_wooden_clicked(false);
    this->ui->checkbox_wooden->setChecked(false); // not sure why the on_checkbox... function is not setting it to false already?!
    on_checkbox_styrofoam_clicked(false);
    this->ui->checkbox_styrofoam->setChecked(false);
    on_checkbox_carton_clicked(false);
    this->ui->checkbox_carton->setChecked(false);
    on_checkbox_plastic_clicked(false);
    this->ui->checkbox_plastic->setChecked(false);
    on_checkbox_fragile_clicked(false);
    this->ui->checkbox_fragile->setChecked(false);
    on_checkbox_dangerous_clicked(false);
    this->ui->checkbox_dangerous->setChecked(false);
    on_checkbox_others_clicked(false);
    this->ui->checkbox_others->setChecked(false);
    updateScenePlanningModels();

}

void BinPackingGUI::on_checkBox_PalletBB_clicked(bool checked)
{
    mutex_PalletBB_.lock();
    if(checked)
        view_PalletBB = true;
    else
        view_PalletBB = false;
    mutex_PalletBB_.unlock();
    updateScenePlanningModels();

}

void BinPackingGUI::on_checkBox_COM_clicked(bool checked)
{
    mutex_BppCOM_.lock();
    if(checked)
        view_BppCOM = true;
    else
        view_BppCOM = false;
    mutex_BppCOM_.unlock();
    updateScenePlanningModels();

}

void BinPackingGUI::on_radioButton_space_opt_clicked(bool checked)
{
    if(checked)
        bpp_opt_goal_ = "opt_space";
    else
        bpp_opt_goal_ = "none";
    updateScenePlanningModels();

}

void BinPackingGUI::on_radioButton_mass_opt_clicked(bool checked)
{
    if(checked)
        bpp_opt_goal_ = "opt_mass";
    else
        bpp_opt_goal_ = "none";
    updateScenePlanningModels();

}

void BinPackingGUI::on_radioButton_com_opt_clicked(bool checked)
{
    if(checked)
        bpp_opt_goal_ = "opt_com";
    else
        bpp_opt_goal_ = "none";
    updateScenePlanningModels();

}

void BinPackingGUI::on_radioButton_default_clicked(bool checked)
{
    if(checked)
        bpp_opt_goal_ = "default";
    updateScenePlanningModels();

}

void BinPackingGUI::resetBppSlot()
{
    this->ui->radioButton_com_opt->setChecked(false);
    this->ui->radioButton_mass_opt->setChecked(false);
    this->ui->radioButton_space_opt->setChecked(false);
    this->ui->radioButton_default->setChecked(true);
    this->bpp_opt_goal_ = "default";
    this->plan_actors_.clear();

    ui->planBtn->setText("Plan");
    ui->progress->setText(" ");
//    ui->progressBar->setValue(0);
    ui->nextBtn->setEnabled(false);

    loadWorldStateSignal(std::string("boxes_loaded"));
//    loadWorldStateSignal(std::string("init"));
    QApplication::processEvents();

    this->on_evalButton_clicked(); //call the wsg to get the BoxPlan, it automatic set the evaluation results to zero;

    // reset the display the ui->listWidget for all actors
    this->ui->listWidget->clear();    

    bpp_actor::QueryActor query;
    query.zone="measurement_area";
    std::vector<bpp_actor::Actor> ma_actors;
    MainWindow::binPackingGUIInstance()->ws_.get_actor_callback(query, ma_actors);

//    wsgtools::WSGClientHelper::getActorsInZone("measurement_area");
    for(bpp_actor::Actor &actor: ma_actors)
    {
        if(actor.type != "static")
        {
//            std::string thumbnailPath = getThumbnailPath("file:///home/nair/data/blob_store/", actor.uuid);
            std::string thumbnailPath = getThumbnailPath("file:///home/nair/data/blob_store/", actor.uuid);


            if(saveFileLocally(actor.thumbnailPath, thumbnailPath))
                this->ui->listWidget->addItem(new QListWidgetItem(QIcon(QString::fromStdString(thumbnailPath.c_str())),actor.uuid.c_str()));
            else
            {
                if(actor.material == "wooden" || actor.material == "Wooden")
                    this->ui->listWidget->addItem(new QListWidgetItem(QIcon(QPixmap(":/images/wooden.png")),actor.uuid.c_str()));
                else if(actor.material == "carton" || actor.material == "Carton")
                    this->ui->listWidget->addItem(new QListWidgetItem(QIcon(QPixmap(":/images/carton.png")),actor.uuid.c_str()));
                else if(actor.material == "styrofoam" || actor.material == "Styrofoam")
                    this->ui->listWidget->addItem(new QListWidgetItem(QIcon(QPixmap(":/images/styrofoam.png")),actor.uuid.c_str()));
                else if(actor.material == "plastic" || actor.material == "Plastic")
                    this->ui->listWidget->addItem(new QListWidgetItem(QIcon(QPixmap(":/images/plastic.png")),actor.uuid.c_str()));
                else if(actor.material == "drum" || actor.material == "Drum")
                    this->ui->listWidget->addItem(new QListWidgetItem(QIcon(QPixmap(":/images/plastic.png")),actor.uuid.c_str()));
                else
                    this->ui->listWidget->addItem(new QListWidgetItem(QIcon(QPixmap(":/images/box_icon_big.png")),actor.uuid.c_str()));
            }
        }
    }
    for(size_t id = 0; id < this->ui->listWidget->count(); id++)
    {
            QListWidgetItem *itm = this->ui->listWidget->item(id);
            itm->setTextColor(Qt::white);
    }
    updateScenePlanningModels();

}

void BinPackingGUI::on_checkBox_support_clicked(bool checked)
{
    if(checked)
        view_support_box = true;
    else
        view_support_box = false;

    updateScenePlanningModels();

}

void BinPackingGUI::on_closeBtn_clicked()
{
    this->ui->listWidget->clearSelection();
    this->ui->stackedWidget->setCurrentIndex(0);
}

void BinPackingGUI::on_defaultviewBtn_clicked()
{
    on_showPlanedPose_clicked(false);
    on_checkBox_PalletBB_clicked(false);
    on_checkBox_COM_clicked(false);
}

void BinPackingGUI::on_plannedviewBtn_clicked(bool checked)
{
    on_showPlanedPose_clicked(checked);
    ui->plannedviewBtn->setChecked(checked);

    updateScenePlanningModels();

}

void BinPackingGUI::on_analyticviewBtn_clicked()
{

}

void BinPackingGUI::on_volumeviewBtn_clicked(bool checked)
{
    on_checkBox_PalletBB_clicked(checked);
    ui->volumeviewBtn->setChecked(checked);

    updateScenePlanningModels();

}

void BinPackingGUI::on_comviewBtn_clicked(bool checked)
{
    on_checkBox_COM_clicked(checked);
    ui->comviewBtn->setChecked(checked);

    updateScenePlanningModels();

}

void BinPackingGUI::on_rppButton_clicked()
{
    QListWidgetItem *item = this->ui->listWidget->currentItem();
    QString string = item->data(Qt::DisplayRole).toString();
    std::string selected_box = string.toStdString();

    emit requestRePalletization(selected_box);
}

void BinPackingGUI::setRppRemoveBoxes(std::string &selected_uuid, std::vector<std::string> &uuids_remove)
{
    // list the boxes which need to be removed
    QMessageBox::StandardButton reply;
    QString message = "To remove "+ QString(selected_uuid.c_str()) + ", we have to depalletize those boxes: \n\n";
    for(int i=0; i < uuids_remove.size(); i++)
        message += QString(uuids_remove[i].c_str()) + "\n";
    reply = QMessageBox::question(this, tr("Re-Palletization"), message, QMessageBox::Yes | QMessageBox::No);

    if(reply == QMessageBox::Yes)
    {
        //move those boxes to measurement area, depalletization
        std::vector<bpp_actor::Actor> boxes_to_replan;
        Eigen::Vector2d current_pos; current_pos << -2.5, -1.0;
        for(std::string uuid : uuids_remove)
        {
            bpp_actor::Actor actor = ws_.get_actor_from_uuid(uuid).front();
            actor.state = "to_pack";
            if(uuid == selected_uuid)
                actor.zone = "others";
            else
                actor.zone = "measurement_area";
            actor.desiredPoseVec[0].position.x = current_pos(0) + actor.bbox.x/2.0;
            actor.desiredPoseVec[0].position.y = current_pos(1) + actor.bbox.y/2.0;
            actor.desiredPoseVec[0].position.z = actor.bbox.z/2.0;
            actor.desiredPoseVec[0].orientation.x = 0.0;
            actor.desiredPoseVec[0].orientation.y = 0.0;
            actor.desiredPoseVec[0].orientation.z = 0.0;
            actor.desiredPoseVec[0].orientation.w = 1.0;
            current_pos(1) += actor.bbox.y + 0.1;
            actor.targetPoseVec[0] = actor.desiredPoseVec[0];

            boxes_to_replan.push_back(actor);
            ws_.update_actor(actor);
            QApplication::processEvents();
            sleep(0.5);
        }
//        wsgtools::WSGClientHelper::updateActors(boxes_to_replan);

        sleep(2);
        // after dpp excution, replan
        reply = QMessageBox::question(this, tr("Re-plan"), "Do you want to re-planning?", QMessageBox::No | QMessageBox::Yes);
        if(reply == QMessageBox::Yes)
        {
            on_closeBtn_clicked();
            on_planBtn_clicked();
        }
    }
    else
    {
        std::cout << "Don't allow to remove boxes!!!\n";
        for(std::string uuid : uuids_remove)
        {
            bpp_actor::Actor actor_back = ws_.get_actor_from_uuid(uuid).front();
            actor_back.desiredPoseVec[0] = actor_back.targetPoseVec[0];
            ws_.update_actor(actor_back);
        }
    }
}

bool BinPackingGUI::saveFileLocallyForce(std::string fileURL, std::string filePath)
{
    std::string fileMem;
    if(!blobhelper::BlobStoreHelper::getBlob(fileURL, fileMem)) return false;
    std::ofstream fileStream(filePath);
    fileStream.write(fileMem.c_str(), fileMem.length());
    fileStream.close();
    return true;
}

bool BinPackingGUI::saveFileLocally(std::string fileURL, std::string filePath, std::string unzipPath, bool checkCache)
{
//    std::cout << "Saving ---> " << fileURL << " " << filePath << std::endl;
//    if(checkCache) if(fileExists(filePath)) return true;
//    std::string fileMem;
//    if(!blobhelper::BlobStoreHelper::getBlob(fileURL, fileMem)) return false;
//    std::ofstream fileStream(filePath);
//    fileStream.write(fileMem.c_str(), fileMem.length());
//    fileStream.close();
//    if(unzipPath.size()){
//        std::string command = "unzip -o " + filePath + " -d " + unzipPath;
//        cout << command << endl;
//        bool ok = system(command.c_str());
//        if(!ok) return false;
//    }
    return true;
}

bool BinPackingGUI::saveFileBlobStore(std::string fileURL, std::string filePath){
    return blobhelper::BlobStoreHelper::addBlob(filePath, fileURL);
}

bool BinPackingGUI::cleanTmpFilesActor(bpp_actor::Actor &actor)
{
    // Faces
    for(size_t faceID = 0; faceID < actor.facesPath.size();++faceID){
        std::string faceFilePath = getFacePath("file:///home/nair/data/blob_store/", actor.uuid, std::to_string(faceID));
        if(fileExists(faceFilePath))
            std::remove(faceFilePath.c_str());
        actor.facesPath[faceID] = getFaceURL(actor.uuid, std::to_string(faceID));
    }

    // Features
    if(actor.featurePointsPath.size())
    {
        if(fileExists(getFeaturePointsPath("file:///home/nair/data/blob_store/", actor.uuid)))
            std::remove(getFeaturePointsPath("file:///home/nair/data/blob_store/", actor.uuid).c_str());
        actor.featurePointsPath = getFeaturePointsURL(actor.uuid);
    }

    // ObjectPoints
    if(actor.objectPointsPath.size())
    {
        if(fileExists(getObjectPointsPath("file:///home/nair/data/blob_store/", actor.uuid)))
            std::remove(getObjectPointsPath("file:///home/nair/data/blob_store/", actor.uuid).c_str());
        actor.objectPointsPath = getObjectPointsURL(actor.uuid);
    }

    // Measure Debug
    if(actor.measureDebugPath.size())
    {
        if(fileExists(getMeasureDebugPath("file:///home/nair/data/blob_store/", actor.uuid)))
            std::remove(getMeasureDebugPath("file:///home/nair/data/blob_store/", actor.uuid).c_str());
        actor.measureDebugPath = getMeasureDebugURL(actor.uuid);
    }

    // Mesh
    if(actor.meshPath.size())
    {
        if(fileExists(getMeshPathZipped("file:///home/nair/data/blob_store/", actor.uuid)))
            std::remove(getMeshPathZipped("file:///home/nair/data/blob_store/", actor.uuid).c_str());
        actor.meshPath = getMeshURLZipped(actor.uuid);
    }

    // Thumbnail
    if(actor.thumbnailPath.size())
    {
        if(fileExists(getThumbnailPath("file:///home/nair/data/blob_store/", actor.uuid)))
            std::remove(getThumbnailPath("file:///home/nair/data/blob_store/", actor.uuid).c_str());
        actor.thumbnailPath = getThumbnailURL(actor.uuid);
    }

    // WRL
    for(size_t body_id = 0;body_id < actor.bodies.size();++body_id){
        if(fileExists(getWRLPath("file:///home/nair/data/blob_store/", actor.uuid, body_id)))
            std::remove(getWRLPath("file:///home/nair/data/blob_store/", actor.uuid, body_id).c_str());
    }

    // Collision WRL
    if(fileExists(getCollisionWRLPath("file:///home/nair/data/blob_store/", actor.uuid)))
        std::remove(getCollisionWRLPath("file:///home/nair/data/blob_store/", actor.uuid).c_str());

    // Texture
    if(actor.body_urls.size()){
        std::string path = "file:///home/nair/data/blob_store/" + actor.uuid + ".wrl";
        std::string urlPath = "file:///home/nair/data/blob_store/" + actor.uuid + ".wrl";
        if(fileExists(path)){
            std::remove(path.c_str());
        }
    }
    if(actor.body_texture_urls.size()){
        std::string path = "file:///home/nair/data/blob_store/" + actor.uuid + ".png";
        std::string urlPath = "file:///home/nair/data/blob_store/" + actor.uuid + ".png";
        if(fileExists(path)){
            std::remove(path.c_str());
        }
    }

    return true;
}

void BinPackingGUI::on_resetButton_clicked()
{
    resetBppSlot();
    resetPPSignal();
    this->ui->nextBtn->setEnabled(false);
    this->ui->rppButton->setEnabled(false);
    this->ui->MASimulationButton->setEnabled(true);

    updateScenePlanningModels();

}


void BinPackingGUI::on_textureviewBtn_clicked(bool checked)
{
//    this->ui->textureviewBtn->setChecked(checked);
    mutex_texture_.lock();
    if(checked)
        view_texture = true;
    else
        view_texture = false;
    mutex_texture_.unlock();

    updateScenePlanningModels();


}

void BinPackingGUI::on_saveButton_clicked()
{
    //save the planed actors to json
    bpp_actor::ActorVec actors = getPlanActors();
    saveBoxesJSON(actors);
}
