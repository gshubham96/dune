//***************************************************************************
// Copyright 2007-2022 Universidade do Porto - Faculdade de Engenharia      *
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
// Author: “gshubham96”                                                     *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

// NMPC Library
// #include <casadi/casadi.hpp>
#include "CourseController.h"

namespace Control
{
  //! Calls the NMPC::CourseController library
  //!
  //! Controls the course of the vehicle
  //! @author “gshubham96”
  //  #TODO Speed model is constant and hardcoded. Fix this.
  namespace ASV
  {
    namespace NmpcCourseController
    {
      using DUNE_NAMESPACES;

      //! %Task arguments.
      struct Arguments
      {
        std::string file_path;
        // config parameters for MPC
        int model_type, cost_type;
        double Tp, Ts;
        // runtime params for MPC
        double Hs, omega_p, gamma, Q, R;
        // solver and output frequency
        double Hz_solver, Hz_output;
      };

      struct Task: public DUNE::Tasks::Task
      {
        // CLASS Vars
        double m_reference_, m_u_opt_;
        std::vector<double> m_theta_;
        std::map<std::string, double> m_config_, m_params_, m_state_;
        Arguments m_args;
        NMPC::CourseController controller;

        // DUNE Vars
        double t_now, t_published, t_solved;
        double time_to_solve, time_to_publish;
        std::string CONTROLLER_STATUS;

        //! Constructor.
        //! @param[in] name task name.
        //! @param[in] ctx context.
        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Tasks::Task(name, ctx)
        {
          param("Model Type", m_args.model_type)
            .defaultValue("1")
            .description("Choose betweem: Linear(1) and Nonlinear(0)");

          param("Cost Type", m_args.cost_type)
            .defaultValue("1")
            .description("Choose between: psi_d(2), dotv(1) and chi_d(0)");

          param("Prediction Horizon", m_args.Tp)
            .defaultValue("50")
            .description("Updates MPC prediction horizon(Tp)");

          param("Discretization Time Period", m_args.Ts)
            .defaultValue("0.5")
            .description("Updates MPC discretization time period(Ts)");

          param("State Cost", m_args.Q)
            .defaultValue("5")
            .description("Updates state cost in objective function");

          param("Input Cost", m_args.R)
            .defaultValue("3.5")
            .description("Updates input cost in objective function");

          param("Wave Height", m_args.Hs)
            .defaultValue("5")
            .description("For Surge Model Prediction");

          param("Peak Wave Frequency", m_args.omega_p)
            .defaultValue("0.6283")
            .description("Specify output frequency in Hz");

          param("Wave Angle of Attack", m_args.gamma)
            .defaultValue("1.57")
            .description("Specify output frequency in Hz");

          param("Solver Rate", m_args.Hz_solver)
            .defaultValue("2")
            .description("Specify solver frequency in Hz");

          param("Output Rate", m_args.Hz_output)
            .defaultValue("5")
            .description("Specify output frequency in Hz");

          // set DUNE params
          time_to_publish = 1/m_args.Hz_output;            
          time_to_solve = 1/m_args.Hz_solver;

          bind<IMC::Abort>(this);
          bind<IMC::EstimatedState>(this);
          bind<IMC::DesiredHeading>(this);
          bind<IMC::AbsoluteWind>(this);
          // bind<IMC::EstimatedFreq>(this);
          bind<IMC::SingleCurrentCell>(this);

          // Making the task activable
          paramActive(Tasks::Parameter::SCOPE_GLOBAL, Tasks::Parameter::VISIBILITY_USER);
          // Initialize entity state.
          setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);

        }

        //! Update internal state with new parameter values.
        void
        onUpdateParameters(void)
        {
          std::map<std::string, double> m_new_configs, m_new_params;
          // checks if MPC config params are changed
          if(paramChanged(m_args.model_type))
            m_new_configs["model_type"] = m_args.model_type;
          if(paramChanged(m_args.cost_type))
            m_new_configs["cost_type"] = m_args.cost_type;
          if(paramChanged(m_args.Tp))
            m_new_configs["Tp"] = m_args.Tp;
          if(paramChanged(m_args.Ts))
            m_new_configs["Ts"] = m_args.Ts;
          if(paramChanged(m_args.Q))
            m_new_params["Q"] = m_args.Q;
          if(paramChanged(m_args.R))
            m_new_params["R"] = m_args.R;
          if(paramChanged(m_args.R))
            m_new_params["Hs"] = m_args.Hs;
          if(paramChanged(m_args.R))
            m_new_params["omega_p"] = m_args.omega_p;
          if(paramChanged(m_args.R))
            m_new_params["gamma"] = m_args.gamma;

          // update MPC configuration
          if(!controller.updateMpcConfig(m_new_configs))
            war("Configuration Parameters NOT Updated!");

          // checks if task params are updated
          if(paramChanged(m_args.Hz_solver))
            time_to_solve = 1/m_args.Hz_solver;
          if(paramChanged(m_args.Hz_output))
            time_to_publish = 1/m_args.Hz_output;            
          
        }

        // #DOUBT Not sure if I need this
        //! Reserve entity identifiers.
        void
        onEntityReservation(void)
        {
        }

        // #DOUBT Not sure if I need this
        //! Resolve entity names.
        void
        onEntityResolution(void)
        {
        }

        //! Acquire resources.
        void
        onResourceAcquisition(void)
        {
        }

        //! Initialize resources.
        void
        onResourceInitialization(void)
        {
          // Update clock
          t_published = 0;
          t_solved = t_published;

          // defined the probelm
          if(!controller.defineMpcProblem())
            cri("Could not define MPC Problem, EXITING!");
          else
            debug("Controller Initialized!");
          t_now = Clock::getSinceEpoch();
        }

        void
        reset(void)
        {
          controller.reset();
          controller.updateMpcConfig(m_config_);
          if(!controller.isProblemConfigured()){
            if(!controller.defineMpcProblem())
              cri("Could not define MPC Problem, EXITING!");
          }
        }

        //! On activation
        void
        onActivation(void)
        {
          setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
        }

        //! On deactivation
        void
        onDeactivation(void)
        {
          setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_IDLE);
        }

        //! Release resources.
        void
        onResourceRelease(void)
        {
        }

        //! Subscribers
        void
        consume(const IMC::Abort* msg)
        {
          if (msg->getDestination() != getSystemId())
            return;

          // This works as redundancy, in case everything else fails
          reset();
          debug("disabling");
        }

        // Updates vehicle state
        void
        consume(const IMC::EstimatedState* msg)
        {
          if (msg->getSource() != getSystemId())
            return;

          // update state
          m_state_["psi"] = msg->psi;
          m_state_["u"] = msg->u;
          m_state_["v"] = msg->v;
          m_state_["r"] = msg->r;

          controller.updateMpcState(m_state_);
          // inf("updated state: %f, %f, %f, %f", m_state_["psi"], m_state_["u"], m_state_["v"], m_state_["r"]);
        }

        // Updated desired course
        void
        consume(const IMC::DesiredHeading* msg)
        {
          m_reference_ = msg->value;
          controller.updateMpcReference(m_reference_);
          debug("updated heading");

        }

        // fill in m_theta_ for wind params
        void consume(const IMC::AbsoluteWind* msg){
          err("wind %f and %f", m_params_["Vw"], m_params_["beta_w"]);

          m_params_["Vw"] = msg->speed;
          m_params_["beta_w"] = msg->dir;
          controller.updateMpcParams(m_params_);
          debug("updated param: wind");

        }

        void
        consume(const IMC::SingleCurrentCell* msg)
        {
          // Find valid measurement closest to surface.
          std::stringstream stream_vel(msg->vel), stream_dir(msg->dir);
          std::vector<double> vels,dirs;

          while(stream_vel.good())
          {
            std::string substr;
            getline(stream_vel, substr, ';');
            vels.push_back(atof(substr.c_str()));
          }
          while(stream_dir.good())
          {
            std::string substr;
            getline(stream_dir, substr, ';');
            dirs.push_back(atof(substr.c_str()));
          }

          m_params_["Vc"] = vels[0];
          m_params_["beta_c"] = dirs[0];
          controller.updateMpcParams(m_params_);
          debug("updated param: current");

        }

        //! publisher function
        void dispatchControl(double u = 1000){
          // IMC Vars
          IMC::SetServoPosition msg;
          msg.value = u;
          cri("dispatching output: %f", u);
          // dispatch(msg);
        } 

        //! Main loop.
        void
        onMain(void)
        {
          // set default parameters
          std::map<std::string, double> params_d;
          params_d["Vc"] = 0.35;      params_d["beta_c"] = 1.57;
          params_d["Vw"] = 5;         params_d["beta_w"] = 1.57;
          params_d["Q"] = 4.5;        params_d["R"] = 1.5;
          params_d["Hs"] = 5; params_d["omega_p"] = 0.6283; params_d["gamma_p"] = 1.57;
          controller.updateMpcParams(params_d);
          while (!stopping())
          {  
            
            // wait to receive messages
            waitForMessages(1.0);

            // get current time
            t_now = Clock::getSinceEpoch();

            // wait till it is time to publish again
            if ((t_now - t_published) < time_to_publish)
              continue;
            
            inf("publishing after : %d", t_now - t_solved);
            // if duration of last solved is greater than threshold
            if((t_now - t_solved) > time_to_solve){
              // solve the problem and check for success
              if(controller.optimizeMpcProblem()){
                t_solved = Clock::getSinceEpoch();
              }
              // else raise an error
              else{
                controller.getErrorString(CONTROLLER_STATUS);
                err("Controller says : %s", CONTROLLER_STATUS.c_str());
              }
            }

            t_now = Clock::getSinceEpoch();
            // publish the latest available solution
            if(controller.getOptimalInput(m_u_opt_, t_now-t_solved)){
              // send input to topic
              dispatchControl(m_u_opt_);
            }
            else{
              controller.getErrorString(CONTROLLER_STATUS);
              err("Controller says : %s", CONTROLLER_STATUS.c_str());
            }
            // update publish time
            inf("publishing after : %d", t_now - t_published);
            t_published = Clock::getSinceEpoch();

          }
        }
      };
    }
  }
}

DUNE_TASK
