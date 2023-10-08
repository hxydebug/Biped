#include "gait_generator.h"
#include <iostream>

Eigen::Vector2d nominal_stance_duration(t_stance, t_stance);
Eigen::Vector2d nominal_stance_dutyrate(0.5, 0.5);
float nominal_contact_detection_phase = 0.1;



gait_generator::gait_generator(){

    stance_duration = nominal_stance_duration;
    stance_dutyrate = nominal_stance_dutyrate;
    swing_duration = stance_duration.array() / stance_dutyrate.array() - stance_duration.array();
    
    initial_leg_state[0] = swing_leg;
    initial_leg_state[1] = stance_leg;
    initial_leg_phase << 0,0;
    // std::cout<<normalized_phase[0]<<std::endl;
    for(int i(0);i<2;i++){
        if(initial_leg_state[i]==swing_leg){
            initial_state_ratio_in_cycle[i] = 1-stance_dutyrate[i];
            next_leg_state[i] = stance_leg;
        }
        else{
            initial_state_ratio_in_cycle[i] = stance_dutyrate[i];
            next_leg_state[i] = swing_leg;
        }
    }
    contact_detection_phase_threshold = nominal_contact_detection_phase;
}

void gait_generator::update(float current_time){

    for(int i(0);i<2;i++){
        float full_cycle_period = (stance_duration[i] / stance_dutyrate[i]);
        float augmented_time = current_time + initial_leg_phase[i]*full_cycle_period;
        float phase_in_full_cycle = fmod(augmented_time,full_cycle_period)/full_cycle_period;
        float ratio = initial_state_ratio_in_cycle[i];
        if(phase_in_full_cycle < ratio){
            desired_leg_state[i] = initial_leg_state[i];
            normalized_phase[i] = phase_in_full_cycle/ratio;
        }
        else{
            desired_leg_state[i] = next_leg_state[i];
            normalized_phase[i] = (phase_in_full_cycle-ratio)/(1-ratio); 
        }

        leg_state[i] = desired_leg_state[i];
        
        // if(normalized_phase[i]<contact_detection_phase_threshold) continue;

        // if(leg_state[i]==swing_leg && contact_state[i]==1){
        //     std::cout<<"early touch down detected."<<std::endl;
        //     leg_state[i] = Early_Contact;
        // }
        // if(leg_state[i]==stance_leg && contact_state[i]==0){
        //     std::cout<<"lost contact detected."<<std::endl;
        //     leg_state[i] = Lose_Contact;
        // }
    }
    // std::cout<<"left: "<<leg_state[0]<<","<<normalized_phase[0]<<std::endl;
    // std::cout<<"righ: "<<leg_state[1]<<","<<normalized_phase[1]<<std::endl;
}
