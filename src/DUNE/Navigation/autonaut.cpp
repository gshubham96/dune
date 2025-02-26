//***************************************************************************
// Copyright 2007-2019 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Faculdade de Engenharia da             *
// Universidade do Porto. For licensing terms, conditions, and further      *
// information contact lsts@fe.up.pt.                                       *
//                                                                          *
// Modified European Union Public Licence - EUPL v.1.1 Usage                *
// Alternatively, this file may be used under the terms of the Modified     *
// EUPL, Version 1.1 only (the "Licence"), appearing in the file LICENCE.md *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// https://github.com/LSTS/dune/blob/master/LICENCE.md and                  *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: Alberto Dallolio                                                 *
//***************************************************************************

// Local headers.
#include <DUNE/Navigation/autonaut.hpp>
#include <DUNE/Math.hpp>

static const double DEG2RAD = M_PI/180.0f;

namespace DUNE
{
  namespace Navigation
  {

    autonaut::autonaut(double T, double dt)
    : m_n_samp(T/dt)
    {
      m_x.resize(m_n_samp);
      m_y.resize(m_n_samp);
      m_psi.resize(m_n_samp);
      m_u.resize(m_n_samp);
      m_v.resize(m_n_samp);
      m_r.resize(m_n_samp);

      // Simulation parameters
      m_T = T;
      m_DT = dt;

      // Model parameters
      m_A = 2.5; // [m]
      m_B = 2.5; // [m]
      m_C = 0.4; // [m]
      m_D = 0.4; // [m]
      m_l = (m_A + m_B);
      m_w = (m_C + m_D); // this is the beam.

      calculate_position_offsets();
    }

    autonaut::~autonaut(){
    }

    void autonaut::linearPredictionInger(const std::vector<double>& state, double u_d, double psi_d)
    {
      m_psi(0) = normalize_angle(psi_d);
	    m_x(0) = state[0] + m_os_x*cos(state[2]) - m_os_y*sin(state[2]);
	    m_y(0) = state[1] + m_os_x*sin(state[2]) + m_os_y*cos(state[2]);
	    m_u(0) = state[3];
	    m_v(0) = state[4];
	    m_r(0) = state[5];

      double r11, r12, r21, r22;

      r11 = cos(psi_d);
      r12 = -sin(psi_d);
      r21 = sin(psi_d);
      r22 = cos(psi_d);

      for (int i = 0; i < m_n_samp-1; i++){

        m_x(i+1) = m_x(i) + m_DT*(r11*m_u(i) + r12*m_v(i));
        m_y(i+1) = m_y(i) + m_DT*(r21*m_u(i) + r22*m_v(i));
        m_psi(i+1) = psi_d;
        m_u(i+1) = u_d;
        m_v(i+1) = 0;

        /*if(i<=m_n_samp-2 && i>= m_n_samp-20)
        {
          std::cout << "X INSIDE= " << m_x(i) << std::endl;
				  std::cout << "Y INSIDE= " << m_y(i) << std::endl;
          std::cout << "PSI INSIDE= " << m_psi(i)*180.0f/M_PI << std::endl;
        }*/
	    }
    }

    void autonaut::linearPrediction(const std::vector<double>& state, double u_d, double psi_d, const Eigen::Matrix<double,-1,2>& waypoints_, double Chi_ca, int course_change_point, int guidance_strategy, double R, double de, double Ki)
    {
      Eigen::Vector2d d_s_wp1, los_s_wp1, d_wp0_wp1, los_wp0_wp1;
      int n_Hd = 0; //50, 200; //getHd()/getDT(); // 30/0.5
      
      //int guidance_strategy = 0; // 0=Line-of-sight (LOS), 1= WP-pursiut (WPP), >1= Course-Hold (CH) 
      
      // LOS guidance dynamics variables and parameters
      //double psi_d0 = normalize_angle(psi_d-Chi_ca);
      double psi_r; // LOS path course correction 
      double e=0, e_integral=0; // cross-track error and its integral
      double max_integral_corr = M_PI*20.0/180.0; //max integral correction pi*20/180
      Eigen::MatrixXd waypoints(waypoints_.rows(), waypoints_.cols()); waypoints = waypoints_;
      
      
      int init_samp = course_change_point*25/m_DT; // assumes course is constant for at least (25s)
      if (guidance_strategy >= 2)  
        init_samp = init_samp*4;
      
      
      // initial prediction state
      if (init_samp == 0)
      {
        m_psi(0) = -normalize_angle(state[2]); // bug fix: use normalized psi // MR interface output sign change
        psi_d = normalize_angle(psi_d); // bug fix: use normalized psi_d
        m_x(0) = state[0] + m_os_x*std::cos(m_psi(0)) - m_os_y*std::sin(m_psi(0));
        m_y(0) = state[1] + m_os_x*std::sin(m_psi(0)) + m_os_y*std::cos(m_psi(0));

        /*psi(0) = normalize_angle(psi_d);
        x(0) = state[0] + os_x*std::cos(psi_d) - os_y*std::sin(psi_d);
        y(0) = state[1] + os_x*std::sin(psi_d) + os_y*std::cos(psi_d);*/
        m_u(0) = state[3];
        m_v(0) = state[4];
        m_r(0) = state[5];

      }
      
      // detect whether waypoints have been switched when init_samp>0, and copy next waypoint	 
      if (init_samp > 0 && guidance_strategy < 2)
      { 	
        d_s_wp1(0) = waypoints(1,0) - m_x(init_samp);  
        d_s_wp1(1) = waypoints(1,1) - m_y(init_samp); 
        los_s_wp1 = d_s_wp1/d_s_wp1.norm();	
        
        d_wp0_wp1(0) = waypoints(1,0) - waypoints(0,0); 
        d_wp0_wp1(1) = waypoints(1,1) - waypoints(0,1); 
        los_wp0_wp1 = d_wp0_wp1/d_wp0_wp1.norm();	
          
        bool leg_passed = los_wp0_wp1.dot(-los_s_wp1) > cos(90*DEG2RAD);		
        
        //std::cout << "leg_passed INSIDE:  " << leg_passed << std::endl;
          
        // update waypoints for the initial sample 
        if (leg_passed){
          // use next waypoints as current waypoints 
          for (int k=0; k < waypoints_.rows()-1; k++){
            waypoints.row(k) = waypoints_.row(k+1);
          }		
        }	
      }
      
      double r11, r12, r21, r22;


      // compute new course
      double psi_path = std::atan2(waypoints(1,1) - waypoints(0,1), 
            waypoints(1,0) - waypoints(0,0));
      //std::cout << "PSI_PATH INSIDE:  " << psi_path*180.0f/M_PI << std::endl;

      // initial along-track error using Eq (10.58 or 10.10) in Fossen 2011
      // NB! the same as remaining track distance  
      //double along_track_dist0 = std::fabs((waypoints(1,0) - m_x(0))*std::cos(psi_path) + (waypoints(1,1) - m_y(0))*std::sin(psi_path));				
      
      // ASV dynamics prediction loop
      
      for (int i = init_samp; i < m_n_samp-1; i++)
      {
                  
          // distance of track from wp0 to wp1
        double track_dist = sqrt(pow(waypoints(1,0) - waypoints(0,0),2) + pow(waypoints(1,1) - waypoints(0,1),2));

        //std::cout << "track_dist INSIDE:  " << track_dist << std::endl;
        
        
        if (track_dist > R + m_A + m_B)
        { // else last WP reached, assume constant course! 
      
          
          // Guidance prediction OPT1: use next wp direction as desired course after t= t_Hd	
          
          //if (1==guidance_strategy && i > n_Hd && (std::fabs(angle_diff(psi_path, psi_d0)) > 15*DEG2RAD || along_track_dist0 < track_dist*0.9) ){ // 0.9 means the asv is on the move to the WP2	
          if (1==guidance_strategy && i > n_Hd)
          { // 0.9 means the asv is on the move to the WP2	
      
            // use pure-pursuit guidance behavior after t= t_Hd?
            d_s_wp1(0) = waypoints(1,0) - m_x(i); 
            d_s_wp1(1) = waypoints(1,1) - m_y(i); 
            
            // use course-hold guidance behavior after t= t_Hd?	
            //d_s_wp1(0) = waypoints(1,0) - waypoints(0,0); 
            //d_s_wp1(1) = waypoints(1,1) - waypoints(0,0); 
                
            los_s_wp1 = d_s_wp1/d_s_wp1.norm();
        
            psi_d = std::atan2(d_s_wp1(1),d_s_wp1(0));
          
            // apply offset
            psi_d = psi_d + Chi_ca;	// approx behavior as input psi_d	
          }
      
      
          // Guidance prediction OPT2: simulate LOS dynamics	
                
          //if (0==guidance_strategy && i > n_Hd && (std::fabs(angle_diff(psi_path, psi_d0)) > 0*DEG2RAD || along_track_dist0 < track_dist*0.9) ){ // 15 deg is the grid size for course offsets 
          if (0==guidance_strategy && i > n_Hd)
          {
            // compute cross-track error Eq (10.59 or 10.10) in Fossen 2011
            e = -(m_x(i) - waypoints(1,0)) * std::sin(psi_path) 
                +(m_y(i) - waypoints(1,1)) * std::cos(psi_path);
            e_integral += e * m_DT;
            
            if (e_integral * Ki > max_integral_corr){
              e_integral -= e * m_DT;
            }
              
            psi_r = std::atan2( -(e + Ki * e_integral) , de);
            psi_d = psi_path + psi_r;
          
            // apply offset
            psi_d = psi_d + Chi_ca;	// approx same behavior as input psi_d
            //if(i<=m_n_samp-2 && i>= m_n_samp-100)
            //    std::cout << "PSI_DESIRED INSIDE:  " << psi_d*180.0f/M_PI << std::endl;
          }

          // compute values for handling waypoint switching events
                  
          // asv-wp radius
          double asv_wp_radius_sqrd = pow(m_x(i) - waypoints(1,0),2) + pow(m_y(i) - waypoints(1,1),2);
          
          // along-track error using Eq (10.58 or 10.10) in Fossen 2011
          // NB! the same as remaining track distance  
          double along_track_dist = std::fabs((waypoints(1,0) - m_x(i))*std::cos(psi_path) + (waypoints(1,1) - m_y(i))*std::sin(psi_path));
                    
      
          // WP switching criteria
          bool switch_wp = 
            // circle of acceptance: (x_wp1 - x)² + (y_wp1 - y)² < R² 
            asv_wp_radius_sqrd < R*R
            ||	
            // progress along path: s_total - s(t) < R or s(t) < R? depends on s(t)
            along_track_dist < R; // track_dist - ?
       
          if (switch_wp)
          {
            // use next waypoints as current waypoints 
            for (int k=0; k < waypoints_.rows()-1; k++){
              waypoints.row(k) = waypoints_.row(k+1);
            }
            
            // reset los integral
            e_integral = 0; 
          
            // compute new course
            psi_path = std::atan2(waypoints(1,1) - waypoints(0,1), 
                waypoints(1,0) - waypoints(0,0));	
          }
		    }

        psi_d = normalize_angle(psi_d);		

        r11 = std::cos(psi_d);
        r12 = -std::sin(psi_d);
        r21 = std::sin(psi_d);
        r22 = std::cos(psi_d);
        
        m_x(i+1) = m_x(i) + m_DT*(r11*m_u(i) + r12*m_v(i));
        m_y(i+1) = m_y(i) + m_DT*(r21*m_u(i) + r22*m_v(i));
        m_psi(i+1) = psi_d;
        m_u(i+1) = u_d;
        m_v(i+1) = 0;

        /*if(i<=m_n_samp-2 && i>= m_n_samp-100)
        {
          std::cout << "X INSIDE= " << m_x(i) << std::endl;
				  std::cout << "Y INSIDE= " << m_y(i) << std::endl;
          std::cout << "PSI INSIDE= " << m_psi(i)*180.0f/M_PI << std::endl;
        }*/

	    }
    }

    Eigen::VectorXd autonaut::getX(){
      return m_x;
    }

    Eigen::VectorXd autonaut::getY(){
      return m_y;
    }

    Eigen::VectorXd autonaut::getPsi(){
      return m_psi;
    }

    Eigen::VectorXd autonaut::getU(){
      return m_u;
    }

    Eigen::VectorXd autonaut::getV(){
      return m_v;
    }

    Eigen::VectorXd autonaut::getR(){
      return m_r;
    }

    double autonaut::getA(){
      return m_A;
    }

    double autonaut::getB(){
      return m_B;
    }

    double autonaut::getC(){
      return m_C;
    }

    double autonaut::getD(){
      return m_D;
    }

    double autonaut::getL(){
      return m_l;
    }

    double autonaut::getW(){
      return m_w;
    }

    double autonaut::getT(){
      return m_T;
    }

    double autonaut::getDT(){
      return m_DT;	
    }

    double autonaut::getNsamp(){
      return m_n_samp;
    }

    void  autonaut::setA(double A){
      m_A = A;
    }

    void autonaut::setB(double B){
      m_B = B;
    }

    void autonaut::setC(double C){
      m_C = C;
    }

    void autonaut::setD(double D){
      m_D = D;
    }


    void autonaut::setT(double T){
      m_T = T;
    }

    void autonaut::setDT(double DT){
      m_DT = DT;
    }

    void autonaut::setNsamp(int n_samp){
      m_n_samp = n_samp;
    }


    void autonaut::calculate_position_offsets(){
      m_os_x = m_A-m_B;
      m_os_y = m_D-m_C;
    }

    // This is not needed, dune has it already.
    double autonaut::normalize_angle(double angle)
    {
      if( std::isinf(angle)) return angle;
      
      while(angle <= -M_PI){
        angle += 2*M_PI;
      }
      
      while (angle > M_PI){
        angle -= 2*M_PI;
      }

      return angle;
    }
  }
}