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

    // drop_server = n.advertiseService("drop_gas", dropControl);

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

MainFilamentSimulator::~MainFilamentSimulator()
{
}

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
    sim.gas_source_pos_z = 1.0;
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

//=============================//
//        Main Part            //
//=============================//
int aliveIndex = -1;
int max_source_num = 50;
double x_coor = 0.00;
double y_coor = 0.00;


void doMsg(const sim_drop_topic::coorMsg::ConstPtr& msg_p){
    // std::string msg = msg_p->data;
    ROS_INFO("Received message: (%f, %f)", msg_p->x, msg_p->y);
    aliveIndex = aliveIndex + 1;
    x_coor = msg_p->x;
    y_coor = msg_p->y;
}

//==============================//
//			MAIN                //
//==============================//
int main(int argc, char **argv)
{
	// Init ROS-NODE
	ros::init(argc, argv, "new_filament_simulator");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<sim_drop_topic::coorMsg>("chatter", 10, doMsg);

    MainFilamentSimulator mfs;
    mfs.cell_size = 0.1;

    CFilamentSimulator sim[max_source_num];

    ros::Duration(2).sleep();

    ROS_INFO("Experiment environment is ready...");
    // ros::Time last_time = ros::Time::now();

    // run the simulation as long as there are alive sources
    int simIndex = -1;
    while(ros::ok()){
        // update the state of gas source
        if(aliveIndex > simIndex){
            // ROS_INFO("001 aliveIndex: %d, simIndex: %d", aliveIndex, simIndex);

            mfs.loadSimulatorData(sim[aliveIndex], aliveIndex);
            sim[aliveIndex].gas_source_pos_x = x_coor;
            sim[aliveIndex].gas_source_pos_y = y_coor;
            sim[aliveIndex].CalParameters();
            sim[aliveIndex].initSimulator();
            sim[aliveIndex].alive = true;
            
            if(aliveIndex < (max_source_num - 1)){
                simIndex = simIndex + 1;
                
            }else{
                simIndex = -1;
                aliveIndex = -1;
            }
            // ROS_INFO("002 aliveIndex: %d, simIndex: %d", aliveIndex, simIndex);
            
        }

        for(int i=0; i<max_source_num; i++){
            if(sim[i].alive){
                sim[i].preMarker();
                mfs.filament_marker.points.insert(mfs.filament_marker.points.end(), 
                                                    sim[i].filament_marker.points.begin(), sim[i].filament_marker.points.end());
                mfs.filament_marker.colors.insert(mfs.filament_marker.colors.end(), 
                                                    sim[i].filament_marker.colors.begin(), sim[i].filament_marker.colors.end());
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