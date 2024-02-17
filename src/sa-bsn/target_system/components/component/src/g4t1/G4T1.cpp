#include "component/g4t1/G4T1.hpp"

#define BATT_UNIT 0.001

using namespace bsn::processor;

G4T1::G4T1(int &argc, char **argv, const std::string &name) : lost_packt(false),
    CentralHub(argc, argv, name, true, bsn::resource::Battery("ch_batt", 100, 100, 1) ),
    patient_status(0.0) {}
	
G4T1::~G4T1() {}

std::vector<std::string> G4T1::getPatientStatus() {
    std::string sensor_risk_str;
    std::string abps;
    std::string abpd;
    std::string oxi;
    std::string ecg;
    std::string trm;
    std::string glc;
    std::string rsp;

    for (int i = 0; i < 7; i++) {
        double sensor_risk = data_buffer[i].back();


        if (sensor_risk > 0 && sensor_risk <= 20) {
            sensor_risk_str = "low risk";
        } else if (sensor_risk > 20 && sensor_risk <= 65) {
            sensor_risk_str = "moderate risk";
        } else if (sensor_risk > 65 && sensor_risk <= 100) {
            sensor_risk_str = "high risk";
        } else {
            sensor_risk_str = "unknown";
        }

        if (i == 0) {
            trm = sensor_risk_str;
            trm_risk = sensor_risk;
        } else if (i == 1) {
            ecg = sensor_risk_str;
            ecg_risk = sensor_risk;
        } else if (i == 2) {
            oxi = sensor_risk_str;
            oxi_risk = sensor_risk;
        } else if (i == 3) {
            abps = sensor_risk_str;
            abps_risk = sensor_risk;
        } else if (i == 4) {
            abpd = sensor_risk_str;
            abpd_risk = sensor_risk;
        } else if (i == 5) {
            glc = sensor_risk_str;
            glc_risk = sensor_risk;
        } else if (i == 6) {
            rsp = sensor_risk_str;
            rsp_risk = sensor_risk;
        }
    }

    std::vector<std::string> v = {trm, ecg, oxi, abps, abpd, glc, rsp};  
    return v;
}

void G4T1::setUp() {
    Component::setUp();

    ros::NodeHandle config;

    double freq;
    config.getParam("frequency", freq);
    rosComponentDescriptor.setFreq(freq);

    for (std::vector<std::list<double>>::iterator it = data_buffer.begin();
        it != data_buffer.end(); ++it) {
            (*it) = {0.0};
    }

    pub = config.advertise<messages::TargetSystemData>("TargetSystemData", 10);
}

void G4T1::tearDown() {}

void G4T1::collect(const messages::SensorData::ConstPtr& msg) {
    int type = getSensorId(msg->type);
    double risk = msg->risk;
    double batt = msg->batt;
    
    battery.consume(BATT_UNIT);
    if (msg->type == "null" || int32_t(risk) == -1)  throw std::domain_error("risk data out of boundaries");

    /*update battery status for received sensor info*/
    if (msg->type == "thermometer") {
        trm_batt = batt;
        trm_raw = msg->data;
    } else if (msg->type == "ecg") {
        ecg_batt = batt;
        ecg_raw = msg->data;
    } else if (msg->type == "oximeter") {
        oxi_batt = batt;
        oxi_raw = msg->data;
    } else if (msg->type == "abps") {
        abps_batt = batt;
        abps_raw = msg->data;
    } else if (msg->type == "abpd") {
        abpd_batt = batt;
        abpd_raw = msg->data;
    } else if (msg->type == "glucosemeter") {
        glc_batt = batt;
        glc_raw = msg->data;
    } else if (msg->type == "respiration") {
        glc_batt = batt;
        glc_raw = msg->data;
    }

    if (buffer_size[type] < max_size) {
        data_buffer[type].push_back(risk);
        buffer_size[type] = data_buffer[type].size();
        total_buffer_size = std::accumulate(std::begin(buffer_size), std::end(buffer_size), 0, std::plus<int>());
    } else {
        data_buffer[type].push_back(risk);
        data_buffer[type].erase(data_buffer[type].begin());//erase the first element to avoid overflow
        lost_packt = true;
    }
}

void G4T1::process(){
    battery.consume(BATT_UNIT * data_buffer.size());
    std::vector<double> current_data;

    for(std::vector<std::list<double>>::iterator it = data_buffer.begin(); it != data_buffer.end(); it++) {
        double el = it->front();
        current_data.push_back(el);
        if(it->size() > 1) it->pop_front();
    }

    patient_status = data_fuse(current_data); // consumes 1 packt per sensor (in the buffers that have packages to data_bufferbe processed)
    for (int i = 0; i < buffer_size.size(); ++i){ // update buffer sizes
        buffer_size[i] = data_buffer[i].size();
    }
    total_buffer_size = std::accumulate(std::begin(buffer_size), std::end(buffer_size), 0, std::plus<int>()); //update total buffer size 

    // std::vector<std::string> risks;
    getPatientStatus();

    std::string patient_risk;

    if(patient_status <= 20) {
        patient_risk = "VERY LOW RISK";
    } else if(patient_status > 20 && patient_status <= 40) {
        patient_risk = "LOW RISK";
    } else if(patient_status > 40 && patient_status <= 60) {
        patient_risk = "MODERATE RISK";
    } else if(patient_status > 60 && patient_status <= 80) {
        patient_risk = "CRITICAL RISK";
    } else if(patient_status > 80 && patient_status <= 100) {
        patient_risk = "VERY CRITICAL RISK";
    }

    ROS_INFO("*****************************************");
    ROS_INFO("PatientStatusInfo#");
    ROS_INFO("| THERM_RISK: %s", std::to_string(trm_risk).c_str());
    ROS_INFO("| ECG_RISK: %s", std::to_string(ecg_risk).c_str());
    ROS_INFO("| OXIM_RISK: %s", std::to_string(oxi_risk).c_str());
    ROS_INFO("| ABPS_RISK: %s", std::to_string(abps_risk).c_str());
    ROS_INFO("| ABPD_RISK: %s", std::to_string(abpd_risk).c_str());
    ROS_INFO("| GLC_RISK: %s", std::to_string(glc_risk).c_str());
    ROS_INFO("| RSP_RISK: %s", std::to_string(rsp_risk).c_str());
    ROS_INFO("| PATIENT_STATE: %s", patient_risk.c_str());
    ROS_INFO("*****************************************");
}

int32_t G4T1::getSensorId(std::string type) {
    if (type == "thermometer")
        return 0;
    else if (type == "ecg")
        return 1;
    else if (type == "oximeter")
        return 2;
    else if (type == "abps")
        return 3;
    else if (type == "abpd")		
        return 4;
    else if (type == "glucosemeter")        
        return 5;
    else if (type == "respiration")        
        return 6;
    else {
        ROS_WARN("UNKNOWN TYPE: %s", type.c_str());
        return -1;
    }
}

void G4T1::transfer() {
    messages::TargetSystemData msg;

    msg.trm_batt = trm_batt;
    msg.ecg_batt = ecg_batt;
    msg.oxi_batt = oxi_batt;
    msg.abps_batt = abps_batt;
    msg.abpd_batt = abpd_batt;
    msg.glc_batt = glc_batt;
    msg.rsp_batt = rsp_batt;

    msg.trm_risk = trm_risk;
    msg.ecg_risk = ecg_risk;
    msg.oxi_risk = oxi_risk;
    msg.abps_risk = abps_risk;
    msg.abpd_risk = abpd_risk;
    msg.glc_risk = glc_risk;
    msg.rsp_risk = rsp_risk;
    
    msg.trm_data = trm_raw;
    msg.ecg_data = ecg_raw;
    msg.oxi_data = oxi_raw;
    msg.abps_data = abps_raw;
    msg.abpd_data = abpd_raw;
    msg.glc_data = glc_raw;
    msg.rsp_data = glc_raw;

    msg.patient_status = patient_status;

    pub.publish(msg);

    if (lost_packt) {
        lost_packt = false;
        throw std::domain_error("lost data due to package overflow");
    }
}