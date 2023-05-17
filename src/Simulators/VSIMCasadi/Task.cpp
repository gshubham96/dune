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
// Author: gshubham96                                                       *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>
using DUNE_NAMESPACES;

namespace Simulators
{
  //! Vehicle SIMulator for DUNE.
  //! %VSIM is responsible for multiple vehicle simulation.
  //! In the present, it is able to simulate
  //! Unmanned Underwater Vehicles
  //! and Autonomous Surface Vehicles.
  //!
  //! @author Shubham Garg
  namespace VSIMCasadi
  {
    //! %Task arguments.
    struct Arguments
    {
      //! Entity label of the stream velocity source.
      std::string svlabel, cvlabel, wvlabel;
      //! Simulation time multiplier
      double time_multiplier;
      //! time step
      double Ts;
      //! model type
      std::string model_type, file_path;

      //! parameters const for debugging only! (change to update from topics/msgs)
      double Vc, Vw, beta_c, beta_w, Hs, omega_p, gamma_p;
    };

    //! Simulator task.
    struct Task: public Tasks::Periodic
    {
      //! Simulation vehicle.
      NmpcDynamics m_simulator;
      //! Simulated position (X,Y,Z).
      IMC::SimulatedState m_state;
      //! Environment Forces
      std::map<std::string, double> m_config_, m_params_;
      std::vector<double> m_vel, m_vel_next;
      //! Task arguments.
      Arguments m_args;
      //! Stream velocity.
      double m_svel[3];
      //! Rudder input, Discretization Time
      double delta, tS;

      Task(const std::string& name, Tasks::Context& ctx):
        Periodic(name, ctx)
      {
        param("Time Multiplier", m_args.time_multiplier)
          .defaultValue("1.0")
          .description("Simulation time multiplier");

        param("Time Step", m_args.Ts)
          .defaultValue("0.1")
          .description("Discretization Time");

        param("File Path", m_args.file_path)
          .defaultValue("system.csv")
          .description("Absolute Path of the system.csv file");

        param("Current Speed", m_args.Vc)
          .defaultValue("0.35")
          .description("Speed of Currents");
        param("Current Direction", m_args.beta_c)
          .defaultValue("1.57")
          .description("Direction of Currents");

        param("Wind Speed", m_args.Vw)
          .defaultValue("5.0")
          .description("Wind Speed");
        param("Wind Direction", m_args.beta_w)
          .defaultValue("1.0")
          .description("Direction of Winds");

        param("Wave Height", m_args.Hs)
          .defaultValue("5.0")
          .description("Peak Wave Height");
        param("Wave Frequency", m_args.omega_p)
          .defaultValue("0.6283")
          .description("Fill");
        param("Wave Direction", m_args.gamma_p)
          .defaultValue("1.57")
          .description("Direction of Propulsion Waves");

        param("Entity Label - Stream Velocity Source", m_args.svlabel)
          .defaultValue("Stream Velocity Simulator")
          .description("Entity label of the stream velocity source.");

        // Register handler routines.
        bind<IMC::GpsFix>(this);
        bind<IMC::ServoPosition>(this);
        // bind<IMC::EstimatedStreamVelocity>(this);
      }

      void
      onUpdateParameters(void)
      {
        // controls sim speed?
        if (m_args.time_multiplier != 1.0)
        {
          Time::Clock::setTimeMultiplier(m_args.time_multiplier);
          war("Using time multiplier: x%.2f", Time::Clock::getTimeMultiplier());
        }

        if(paramChanged(m_args.Ts))
          tS = m_args.Ts;

        // Currents
        if(paramChanged(m_args.Vc))
          m_params_["Vc"] = m_args.Vc;
        if(paramChanged(m_args.beta_c))
          m_params_["beta_c"] = m_args.beta_c;

        // Winds
        if(paramChanged(m_args.Vw))
          m_params_["Vw"] = m_args.Vw;
        if(paramChanged(m_args.beta_w))
          m_params_["beta_w"] = m_args.beta_w;

        // Waves
        if(paramChanged(m_args.Hs))
          m_params_["Hs"] = m_args.Hs;
        if(paramChanged(m_args.omega_p))
          m_params_["omega_p"] = m_args.omega_p;
        if(paramChanged(m_args.gamma_p))
          m_params_["gamma_p"] = m_args.gamma_p;

      }

      //! Release allocated resources.
      void
      onResourceRelease(void)
      {
      }

      //! Initialize resources and add vehicle to the world.
      void
      onResourceInitialization(void)
      {
        m_simulator.configureDynamics("nonlinear", m_args.file_path, tS, false);
        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
        requestActivation();

        m_vel = std::vector<double> param_vector(4, 0);

      }

      void
      consume(const IMC::GpsFix* msg)
      {
        if (msg->type != IMC::GpsFix::GFT_MANUAL_INPUT)
          return;

        // Define vehicle origin.
        m_state.lat = msg->lat;
        m_state.lon = msg->lon;
        m_state.height = msg->height;

        requestActivation();
      }

      void
      consume(const IMC::ServoPosition* msg)
      {
        delta = msg->value;
      }

      void
      task(void)
      {
        debug("1");

        // inputs == m_state_, delta, m_params_, m_state_next
        m_simulator.simulateDynamics(m_vel, delta, m_params_, m_vel_next);

        debug("2");
        m_vel = m_vel_next;
        debug(m_vel);

        // Fill attitude.
        m_state.psi = Angles::normalizeRadian(m_vel[0]);

        debug("3");
        // position
        m_state.x += tS*m_vel[1];
        m_state.y += tS*m_vel[2];

        debug("4");
        // Fill angular velocity.
        m_state.r = m_vel[3];

        // Fill linear velocity.
        m_state.u = m_vel[1];
        m_state.v = m_vel[2];

        debug("5");
        // // Fill stream velocity.
        // m_state.svx = m_svel[0];
        // m_state.svy = m_svel[1];
        // m_state.svz = m_svel[2];

        debug("6");
        debug("state - %f, %f, %f, %f", m_state.psi, m_state.u, m_state.v, m_state.r);
        dispatch(m_state);

      }
    };
  }
}

DUNE_TASK
