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
// Author: Shubham Garg                                                     *
//***************************************************************************

#ifndef DUNE_CONTROL_NMPC_COURSE_HPP_INCLUDED_
#define DUNE_CONTROL_NMPC_COURSE_HPP_INCLUDED_

// DUNE headers.
#include <DUNE/Tasks/Task.hpp>
#include <DUNE/Control/NmpcDynamics.hpp>

#include <iostream>
#include <fstream>
#include <iomanip>
#include <filesystem>
#include <casadi/casadi.hpp>

#include <random>
#include <algorithm>
// floor
#include <cmath>    

#define PI M_PI
#define EPS 1e-9
#define DEG2RAD(angle) ((angle) * M_PI / 180.0)
#define RAD2DEG(angle) ((angle) * 180.0 / M_PI)

namespace fs = std::filesystem;

namespace DUNE
{
    namespace Control
    {
        // Export DLL Symbol.
        class DUNE_DLL_SYM NmpcCourse;

        class NmpcCourse: public NmpcDynamics {
            
            private:
                // ##################################
                // ##-------MEMBER VARIABLES-------##
                // ##################################
                // flag to check if valid solution exists
                bool solution_exists_;
                // initialization variable
                int initialized_;
                // MPC Parameters
                int cost_type_;
                double Tp_;
                // course reference for the controller
                double reference_;
                // config parameters, runtime paramters and state for MPC
                std::map<std::string, double> params_, state_;
                // initial guess for warm start
                std::map<std::string, std::vector<double>> args_;
                // NLP Solver
                casadi::Function solver_;
                // optimized input trajectory
                std::vector<double> optimized_vars_, input_traj_;
                // file handling
                int filecount_;
                std::ofstream file_;
                std::string filename_;
                
                // ##################################
                // ##-------MEMBER FUNCTIONS-------##
                // ##################################

                // Function to load defaults for config, params and system dynamics
                bool loadDefaults();

                // cosntructs a mpc-friendly format parameter vector
                std::vector<double> reWriteParams();

                // generates random vector for warm start
                std::vector<double> generateRandomVector(int n);

            public:
                // Function to define and compile the NLP Optimization Problem
                bool defineMpcProblem(bool compile);

                // updates config parameters if user wants to change the NLP
                bool updateMpcConfig(const std::map<std::string, double> &config);

                // updates parameters such as wind, currents, etc
                // need to do it atlease once
                bool updateMpcParams(const std::map<std::string, double> &param);

                // updates mpc state
                bool updateMpcState(const std::map<std::string, double> &state);

                // updates course reference angle [rad]
                bool updateMpcReference(const double &reference);

                // Optimize the NLP with updated state and parameters
                bool optimizeMpcProblem();

                // Get the result from the optimized input
                bool getOptimalInput(double &u_star, const double &t_elapsed);

                // debug function to save trajectory to file
                void saveTrajectoryToFile(void);

                // debug function to print trajectory and other info on screen
                void print_details(void);

                // resets the controller
                void reset(void);

                // checks if the problem is configured properly. If not, configure it
                bool isProblemConfigured();

                // returns error or updates from controller
                void getErrorString(std::string &err);

            // allow user to skip problem configuration
            NmpcCourse(std::string model_type, std::string cost_type, double Tp, double Ts, bool compile);

            // Default Constructor
            NmpcCourse();

            // Destructor
            ~NmpcCourse();

        };
    }
}

#endif