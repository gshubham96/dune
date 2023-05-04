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

#ifndef DUNE_CONTROL_NMPC_HPP_INCLUDED_
#define DUNE_CONTROL_NMPC_HPP_INCLUDED_

// DUNE headers.
#include <DUNE/Tasks/Task.hpp>

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
        namespace NMPC
        {
            // Export DLL Symbol.
            class DUNE_DLL_SYM Dynamics;

            class Dynamics 
            {

            private:
            // ##################################
            // ##-------MEMBER VARIABLES-------##
            // ##################################
            // initialization variable
            int initialized;
            // lengths of state, input and paramter vectors
            int nx, nu, np, N;
            // config params for RK4 simulation
            int model_type_;
            double Ts_; 
            // surge speed model
            std::vector<double> speed_model;
            // get system parameters from "mat" file and load here
            std::map<std::string, double> system_;
            std::string file_path;
            // dynamics function
            casadi::Function x_dot;
            // error handling
            std::string ERROR_STRING;
            
            // ##################################
            // ##-------MEMBER FUNCTIONS-------##
            // ##################################

            // wrap the angle between [-pi, pi]
            double ssa(double diff);
            casadi::SX ssa(casadi::SX diff);

            // reads data from file and stores in passed arg
            bool loadDefaultsFromFile(const std::string &file_name, std::map<std::string, double> &data_from_file);

            // performs sanity check of config params
            bool areParamsSane(const std::map<std::string, double> &mapped_dict);

            public:
            // Function to define and compile the NLP Optimization Problem
            bool defineDynamicsProblem(bool compile);

            //Simulate the dynamics for one time step
            // params keys: Vc, beta_c, Vw, beta_w, Hs, omega_p, gamma_p
            // state_(init/next) keys: psi, u, v, r
            bool simulateDynamics(const std::vector<double> &state_init, const double u0, const std::map<std::string, double> &params, std::vector<double> &state_next);

            // returns error or updates from controller
            void getErrorString(std::string &err);

            // allow user to skip problem configuration
            Dynamics(std::string model_type, double Ts, bool compile);

            // Destructor
            ~Dynamics();

            };

        }
    }
}

#endif