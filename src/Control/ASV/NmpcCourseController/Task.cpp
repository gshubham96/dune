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
#include "CourseController.h"

namespace Control
{
  //! Calls the NMPC::CourseController library
  //!
  //! Controls the course of the vehicle
  //! @author “gshubham96”
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
        // int nx, nu, np;
        int model_type, cost_type;
        double Tp, Ts, Q, R;
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
        double t_now, t_last_solved;
        double solver_rate, output_rate;

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

          param("State Cost", m_args.Q)
            .defaultValue("5")
            .description("Updates state cost in objective function");

          param("Input Cost", m_args.R)
            .defaultValue("3.5")
            .description("Updates input cost in objective function");

          param("Prediction Horizon", m_args.Tp)
            .defaultValue("50")
            .description("Updates MPC prediction horizon(Tp)");

          param("Discretization Time Period", m_args.Ts)
            .defaultValue("0.5")
            .description("Updates MPC discretization time period(Ts)");

          param("Solver Rate", m_args.Hz_solver)
            .defaultValue("2")
            .description("Specify solver frequency in Hz");

          param("Output Rate", m_args.Hz_output)
            .defaultValue("5")
            .description("Specify output frequency in Hz");

          if(!controller.updateMpcConfig(m_config_))
            err("Configuration Parameters NOT Set!");

          bind<IMC::Abort>(this);
          bind<IMC::EstimatedState>(this);
          bind<IMC::DesiredHeading>(this);
          bind<IMC::AbsoluteWind>(this);
          bind<IMC::EstimatedFreq>(this);
          // #DOUBT Not sure if I need this
          // bind<IMC::CurrentProfile>(this);
          // bind<IMC::Rpm>(this);
          // bind<IMC::ControlLoops>(this);

          // #DOUBT Not sure if I need this
          // Initialize entity state.
          setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_IDLE);

        }

        //! Update internal state with new parameter values.
        void
        onUpdateParameters(void)
        {
          // checks if MPC config params are changed
          if(paramChanged(m_args.model_type))
            m_config_["model_type"] = m_args.model_type;
          if(paramChanged(m_args.cost_type))
            m_config_["cost_type"] = m_args.cost_type;
          if(paramChanged(m_args.Tp))
            m_config_["Tp"] = m_args.Tp;
          if(paramChanged(m_args.Ts))
            m_config_["Ts"] = m_args.Ts;
          if(paramChanged(m_args.Q))
            m_config_["Q"] = m_args.Q;
          if(paramChanged(m_args.R))
            m_config_["R"] = m_args.R;

          // update MPC configuration
          if(!controller.updateMpcConfig(m_config_))
            war("Configuration Parameters NOT Updated!");

          // checks if task params are updated
          if(paramChanged(m_args.Hz_solver))
            solver_rate = m_args.Hz_solver;
          if(paramChanged(m_args.Hz_output))
            output_rate = m_args.Hz_output;            
          
        }

        // #DOUBT Not sure if I need this
        //! Reserve entity identifiers.
        // void
        // onEntityReservation(void)
        // {
        // }

        // #DOUBT Not sure if I need this
        //! Resolve entity names.
        // void
        // onEntityResolution(void)
        // {
        // }

        //! Acquire resources.
        void
        onResourceAcquisition(void)
        {
        }

        //! Initialize resources.
        void
        onResourceInitialization(void)
        {
          reset();
        }

        void
        reset(void)
        {
          controller.reset();
          controller.updateMpcConfig(m_config_);
          if(!controller.isProblemConfigured()){
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

          // #DOUBT what does this do?
          if (!isActive())
          {
            return;
          }

          // update state
          m_state_["psi"] = msg->psi;
          m_state_["u"] = msg->u;
          m_state_["v"] = msg->v;
          m_state_["r"] = msg->r;

          controller.updateMpcState(m_state_);
        }

        // Updated desired course
        void
        consume(const IMC::DesiredHeading* msg)
        {
          if (!isActive())
            return;

          m_reference_ = msg->value;
          controller.updateMpcReference(m_reference_);
          debug("DH %f",Angles::degrees(m_reference_));
        }

        // fill in m_theta_ for wind params
        void consume(const IMC::AbsoluteWind* msg){
          if (!isActive())
            return;

          m_params_["Vw"] = msg->speed;
          m_params_["beta_w"] = msg->dir;
          controller.updateMpcParams(m_params_);

        }

        // #DOUBT I am not sure if this is the right message
        // fill in m_theta_ for current params
        // void consume(const IMC::CurrentProfile* msg){
        //   if (!isActive())
        //     return;

        //     m_params_["Vc"] = msg->vel;
        //     m_params_["beta_c"] = msg->dir;
        //     controller.updateMpcParams(m_params_);

        // }

        // fill in m_theta_ for wave foils params
        void consume(const IMC::EstimatedFreq* msg){
          if (!isActive())
            return;

          // #DOUBT where can i get the three wave parameters?
          // m_theta_ = [Hs, Tp, gamma_w]
          m_theta_[0] = msg->value;          

        }

        //! helper function to get surge coefficients
        void computeSurgeCoefficients(){         

          // param_list = Hs, omega_p, gamma, 0, Vc*cos(beta_c), 1 
          std::vector<double> param_list = {m_theta_[0], m_theta_[1], cos(m_theta_[2]), 0, m_params_["Vc"]*cos(m_params_["beta_c"]), 1};
          std::vector<double> speed_model = {0.116392998053662, 0.214487083945715, 0.0880678632611925, -0.00635496887217675, 0.0937464223577265, 0.238364678400396 };        

          double k1 =  0, k2 = 0;
          for(int i = 0; i < 6; i++)
            k1 += param_list[i]*speed_model[i];
          k2 = m_params_["Vw"] * speed_model[3];

          m_params_["k_1"] = k1;
          m_params_["k_2"] = k2;
          controller.updateMpcParams(m_params_);

        }  

        //! publisher function
        void dispatchControl(double u = 1000){
          cri("NLP SOLVER HASN'T RUN FOR A WHILE, SOMETHING IS WRONG");
        } 

        //! Main loop.
        void
        onMain(void)
        {
          while (!stopping())
          {
            waitForMessages(1.0);
            t_now = Clock::getSinceEpoch();

            // Check if time elapsed is greater than threshold
            if((t_now - t_last_solved) > 1/solver_rate){
              // optimize problem and check for success
              if(!controller.optimizeMpcProblem()){
                err("SOLVER FAILED!!");
                spew("did you update the state?");
              }
              else
                t_last_solved = Clock::getSinceEpoch();
            }
            // if not enough time has elapsed, update using existing solution
            else
              m_u_opt_ = controller.getOptimalInput();

            // send input to topic
            dispatchControl(m_u_opt_);

          }
        }
      };
    }
  }
}

DUNE_TASK
