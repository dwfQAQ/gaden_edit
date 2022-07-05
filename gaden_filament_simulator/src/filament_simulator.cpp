#include "filament_simulator/filament_simulator.h"

MainFilamentSimulator::MainFilamentSimulator(){

    loadNodeParams();

    if (save_results && !boost::filesystem::exists(results_location)){
		if (!boost::filesystem::create_directories(results_location))
           ROS_ERROR("[filament] Could not create result directory: %s", results_location.c_str());
		
       	if (!boost::filesystem::create_directories(boost::str(boost::format("%s/wind") % results_location) ) )
           ROS_ERROR("[filament] Could not create result directory: %s", results_location.c_str());
	}
    //Set Publishers and Subscribers
    marker_pub = n.advertise<visualization_msgs::Marker>("filament_visualization", 1);
    serviceGas = n.advertiseService("/odor_value", &MainFilamentSimulator::get_gas_value_srv, this);

    // Wait preprocessing Node to finish?
    preprocessing_done = false;
    if(wait_preprocessing)
    {
        prepro_sub = n.subscribe("preprocessing_done", 1, &MainFilamentSimulator::preprocessingCB, this);
        while(ros::ok() && !preprocessing_done)
        {
            ros::Duration(0.5).sleep();
            ros::spinOnce();
            if (verbose) ROS_INFO("[filament] Waiting for node GADEN_preprocessing to end.");
        }
    }

    filament_marker.header.frame_id = fixed_frame;
	filament_marker.ns = "filaments";
	filament_marker.action = visualization_msgs::Marker::ADD;
	filament_marker.id = 0;
	filament_marker.type = visualization_msgs::Marker::POINTS;
	filament_marker.color.a = 1;
}

MainFilamentSimulator::~MainFilamentSimulator(){}

void MainFilamentSimulator::loadNodeParams(){
    ros::NodeHandle private_nh0("~");
    private_nh0.param<int>("number_of_sources", number_of_sources, 0);
    private_nh0.param<bool>("verbose", verbose, false);  // Verbose
    private_nh0.param<bool>("wait_preprocessing", wait_preprocessing, false);    // Wait PreProcessing
	private_nh0.param<std::string>("fixed_frame", fixed_frame, "/map");  //fixed frame (to disaply the gas particles on RVIZ)

    private_nh0.param<int>("save_results", save_results, 1); // Simulation results.
	private_nh0.param<std::string>("results_location", results_location, "");
    private_nh0.param<double>("results_min_time", results_min_time, 0.0);    //create a sub-folder for this specific simulation
    private_nh0.param<double>("results_time_step", results_time_step, 1.0);
}

void MainFilamentSimulator::preprocessingCB(const std_msgs::Bool& b)
{
    preprocessing_done = true;
}


void MainFilamentSimulator::loadSimulatorData(CFilamentSimulator& sim, int idx){
    ros::NodeHandle private_nh("~");
    // load customized data
    private_nh.param<bool>("verbose", verbose, false);  // Verbose
    private_nh.param<bool>("wait_preprocessing", wait_preprocessing, false);    // Wait PreProcessing
	private_nh.param<double>("sim_time", max_sim_time, 20.0);   // Simulation Time (sec)
	private_nh.param<double>("time_step", time_step, 1.0);  // Time increment between Gas snapshots (sec)
	private_nh.param<int>("num_filaments_sec", numFilaments_sec, 100);  // Num of filaments/sec
    private_nh.param<bool>("variable_rate", variable_rate, false);
    private_nh.param<int>("filament_stop_steps", filament_stop_steps, 0);
	private_nh.param<double>("ppm_filament_center", filament_ppm_center, 20);   // Gas concentration at the filament center - 3D gaussian [ppm]
	private_nh.param<double>("filament_initial_std", filament_initial_std, 1.5);    // [cm] Sigma of the filament at t=0-> 3DGaussian shape
	private_nh.param<double>("filament_growth_gamma", filament_growth_gamma, 10.0); // [cm²/s] Growth ratio of the filament_std
	private_nh.param<double>("filament_noise_std", filament_noise_std, 0.1);    // [cm] Sigma of the white noise added on each iteration
	private_nh.param<int>("gas_type", gasType, 1);  // Gas Type ID
	private_nh.param<double>("temperature", envTemperature, 298.0); // Environment temperature (necessary for molecules/cm3 -> ppm)
	private_nh.param<double>("pressure", envPressure, 1.0); // Enviorment pressure (necessary for molecules/cm3 -> ppm)
	private_nh.param<int>("concentration_unit_choice", gasConc_unit, 1);    // Gas concentration units (0= molecules/cm3,  1=ppm)
	private_nh.param<std::string>("wind_data", wind_files_location, "");    //CFD wind files location	
	private_nh.param<double>("wind_time_step", windTime_step, 1.0); //(sec) Time increment between Wind snapshots --> Determines when to load a new wind field
    private_nh.param<bool>("allow_looping", allow_looping, false);  // Loop
    private_nh.param<int>("loop_from_step", loop_from_step, 1);
    private_nh.param<int>("loop_to_step", loop_to_step, 100);
	private_nh.param<std::string>("occupancy3D_data", occupancy3D_data, "");    // Occupancy gridmap 3D location
	private_nh.param<std::string>("fixed_frame", fixed_frame, "/map");  //fixed frame (to disaply the gas particles on RVIZ)

    // std::string paramNameX = boost::str( boost::format("source_%i_position_x") % idx);
    // std::string paramNameY = boost::str( boost::format("source_%i_position_y") % idx);
    // // std::string paramNameZ = boost::str( boost::format("source_%i_position_z") % idx);
    // private_nh.param<double>(paramNameX.c_str(), gas_source_pos_x, 1.0);   //Source postion (x,y,z)
	// private_nh.param<double>(paramNameY.c_str(), gas_source_pos_y, 1.0);
	// private_nh.param<double>("source_0_position_z", gas_source_pos_z, 1.0);

    // set data to simulator
    sim.verbose = verbose;
    sim.wait_preprocessing = wait_preprocessing;
    sim.max_sim_time = max_sim_time;
    sim.time_step = time_step;
    sim.numFilaments_sec = numFilaments_sec;
    sim.variable_rate = variable_rate;
    sim.filament_stop_steps = filament_stop_steps;
    sim.filament_ppm_center = filament_ppm_center;
    sim.filament_initial_std = filament_initial_std;
    sim.filament_growth_gamma = filament_growth_gamma;
    sim.filament_noise_std = filament_noise_std;
    sim.gasType = gasType;
    sim.envTemperature = envTemperature;
    sim.envPressure = envPressure;
    sim.gasConc_unit = gasConc_unit;
    sim.wind_files_location = wind_files_location;
    sim.windTime_step = windTime_step;
    sim.allow_looping = allow_looping;
    sim.loop_from_step = loop_from_step;
    sim.loop_to_step = loop_to_step;
    sim.occupancy3D_data = occupancy3D_data;
    sim.fixed_frame = fixed_frame;
    sim.gas_source_pos_x = 1.0;
    sim.gas_source_pos_y = 1.0;
    sim.gas_source_pos_z = 0.0;
}



void MainFilamentSimulator::publish_markers(){
    filament_marker.header.stamp = ros::Time::now();
	filament_marker.pose.orientation.w = 1.0;

	//width of points: scale.x is point width, scale.y is point height
	filament_marker.scale.x = cell_size/4;
	filament_marker.scale.y = cell_size/4;
	filament_marker.scale.z = cell_size/4;

    marker_pub.publish(filament_marker);

    filament_marker.points.clear();
	filament_marker.colors.clear();
}

void MainFilamentSimulator::get_gas_concentration(double x, double y, double z){
    gas_conc[0] = 0.0;
    gas_conc[1] = 0.0;
    for(auto it = filaments_0.begin(); it!=filaments_0.end(); it++){
        CFilament fil = *it;
        if(it->valid){
            double dist = sqrt((x-fil.pose_x)*(x-fil.pose_x) + (y-fil.pose_y)*(y-fil.pose_y) + (z-fil.pose_z)*(z-fil.pose_z));
            double limitDistance = fil.sigma*5/100;
            if(dist < limitDistance){
                double distance_cm = 100 * dist;
                double num_moles_target_cm3 = (total_moles_in_filament /
                    (sqrt(8*pow(M_PI,3)) * pow(fil.sigma,3) )) * exp( -pow(distance_cm,2)/(2*pow(fil.sigma,2)) );
                double ppm = num_moles_target_cm3/num_moles_all_gases_in_cm3 * 1000000; //parts of target gas per million
                gas_conc[0] += ppm;   
            }
        }
    }

    for(auto it = filaments_1.begin(); it!=filaments_1.end(); it++){
        CFilament fil = *it;
        if(it->valid){
            double dist = sqrt((x-fil.pose_x)*(x-fil.pose_x) + (y-fil.pose_y)*(y-fil.pose_y) + (z-fil.pose_z)*(z-fil.pose_z));
            double limitDistance = fil.sigma*5/100;
            if(dist < limitDistance){
                double distance_cm = 100 * dist;
                double num_moles_target_cm3 = (total_moles_in_filament /
                    (sqrt(8*pow(M_PI,3)) * pow(fil.sigma,3) )) * exp( -pow(distance_cm,2)/(2*pow(fil.sigma,2)) );
                double ppm = num_moles_target_cm3/num_moles_all_gases_in_cm3 * 1000000; //parts of target gas per million
                gas_conc[1] += ppm;   
            }
        }
    }
}

bool MainFilamentSimulator::get_gas_value_srv(gaden_filament_simulator::GasPosition::Request &req, gaden_filament_simulator::GasPosition::Response &res){
    
    get_gas_concentration(req.x, req.y, req.z);
    res.gas_conc.clear();
    res.gas_type.clear();
    
    res.gas_type.push_back("ethanol");
    res.gas_conc.push_back(gas_conc[0]);
    res.gas_type.push_back("acetone");
    res.gas_conc.push_back(gas_conc[1]);
    ROS_INFO("ethanol: %f, acetone: %f", gas_conc[0], gas_conc[1]);

    return true;
}

//=============================//
//        Main Part            //
//=============================//
int aliveIndex = -1;
int max_source_num = 50;
struct 
{
    int gas_type;
    double x_coor;
    double y_coor;
}gas_drop;


void doMsg(const sim_drop_topic::coorMsg::ConstPtr& msg_p){
    // std::string msg = msg_p->data;
    ROS_INFO("Dispense %s drop at: (%f, %f)", msg_p->type.c_str(), msg_p->x, msg_p->y);
    aliveIndex = aliveIndex + 1;
    // ethanol: 0   ammonia: 1
    if((msg_p->type).compare("ethanol") == 0){
        gas_drop.gas_type = 0;
    }else{
        gas_drop.gas_type = 4;
    }
    gas_drop.x_coor = msg_p->x;
    gas_drop.y_coor = msg_p->y;
}

//==============================//
//			MAIN                //
//==============================//
int main(int argc, char **argv)
{
	// Init ROS-NODE
	ros::init(argc, argv, "new_filament_simulator");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<sim_drop_topic::coorMsg>("drop_coor", 10, doMsg);
    
    MainFilamentSimulator mfs;
    mfs.cell_size = 0.1;

    CFilamentSimulator sim[max_source_num];

    ros::Duration(2).sleep();

    ROS_INFO("Experiment environment is ready...");

    // run the simulation as long as there are alive sources
    int simIndex = -1;
    while(ros::ok()){
        // update the state of gas source
        if(aliveIndex > simIndex){

            mfs.loadSimulatorData(sim[aliveIndex], aliveIndex);
            sim[aliveIndex].gas_source_pos_x = gas_drop.x_coor;
            sim[aliveIndex].gas_source_pos_y = gas_drop.y_coor;
            sim[aliveIndex].gasType = gas_drop.gas_type;
            sim[aliveIndex].CalParameters();
            sim[aliveIndex].initSimulator();
            sim[aliveIndex].alive = true;
            mfs.total_moles_in_filament = sim[aliveIndex].filament_numMoles_of_gas;
            mfs.num_moles_all_gases_in_cm3 = sim[aliveIndex].num_moles_all_gases_in_cm3;

            if(aliveIndex < (max_source_num - 1)){
                simIndex = simIndex + 1;
                
            }else{
                simIndex = -1;
                aliveIndex = -1;
            }
            
        }
        mfs.filaments_0.clear();
        mfs.filaments_1.clear();
        for(int i=0; i<max_source_num; i++){
            if(sim[i].alive){
                sim[i].preMarker();
                mfs.filament_marker.points.insert(mfs.filament_marker.points.end(), 
                                                    sim[i].filament_marker.points.begin(), sim[i].filament_marker.points.end());
                mfs.filament_marker.colors.insert(mfs.filament_marker.colors.end(), 
                                                    sim[i].filament_marker.colors.begin(), sim[i].filament_marker.colors.end());

                if(sim[i].gasType == 0){
                    mfs.filaments_0.insert(mfs.filaments_0.end(), sim[i].filaments.begin(), sim[i].filaments.end()); 
                }else{
                    mfs.filaments_1.insert(mfs.filaments_1.end(), sim[i].filaments.begin(), sim[i].filaments.end()); 
                }
                                                  
                sim[i].postMarker();
            }
        } 

        mfs.publish_markers();
           
        ros::Duration(mfs.time_step).sleep();
        ros::spinOnce();
    }
}

        // if(ros::Time::now() - last_time >= ros::Duration(3)){
        //     if(aliveIndex < max_source_num){
        //         aliveIndex++;
        //     }
        //     last_time = ros::Time::now();
        // }

// if ( (sim.save_results==1) && (sim.sim_time>=sim.results_min_time) )
        // {
        //     if ( floor(sim.sim_time/sim.results_time_step) != sim.last_saved_step ){
        //         sim.save_state_to_file();
        //     }
        // }

        //Create simulator obj array and initialize it
    
    // CFilamentSimulator sim;
    // CFilamentSimulatorVector.push_back(sim);
	// CFilamentSimulator sim[max_source_num];    

    // for(int i=0; i<max_source_num; i++){
    //     mfs.loadSimulatorData(sim[i], i);
    //     sim[i].CalParameters();
    //     sim[i].initSimulator();
    //     // sim[i].alive = true;
    // }

    // mfs.loadSimulatorData(sim[aliveIndex], aliveIndex);
    // sim[aliveIndex].gas_source_pos_x = x_coor;
    // sim[aliveIndex].gas_source_pos_y = y_coor;
    // sim[aliveIndex].CalParameters();
    // sim[aliveIndex].initSimulator();
            
    // sim[aliveIndex].alive = true;
    // for(int i=0; i<max_source_num; i++){
    //     if(sim[i].alive){
    //         sim[i].preMarker();
    //         mfs.filament_marker.points.insert(mfs.filament_marker.points.end(), 
    //                                             sim[i].filament_marker.points.begin(), sim[i].filament_marker.points.end());
    //         mfs.filament_marker.colors.insert(mfs.filament_marker.colors.end(), 
    //                                             sim[i].filament_marker.colors.begin(), sim[i].filament_marker.colors.end());
    //         sim[i].postMarker();
    //     }
    // }  