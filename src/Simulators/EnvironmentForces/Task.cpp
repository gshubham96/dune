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
// Author: Alberto Dallolio                                                 *
//***************************************************************************

// ISO C++ 98 headers.
#include <iomanip>

// DUNE headers.
#include <DUNE/DUNE.hpp>

using DUNE_NAMESPACES;

namespace Simulators
{
  namespace EnvironmentForces
  {
    struct Arguments
    {
      //! Transmission frequency in seconds.
      double transmission_period;
      //! Measure wind.
      bool wind;
      //! Wind direction (deg) and speed (m/s).
      double wind_dir, wind_vel;
      //! Wind direction and speed standard deviations.
      double stdev_wind_dir, stdev_wind_vel;
      //! Measure current.
      bool current;
      //! Current depth (m), direction (deg) and speed (m/s).
      double curr_depth, curr_dir, curr_vel;
      //! Current direction and speed standard deviations.
      double stdev_curr_dir, stdev_curr_vel;
      //! Measure waves.
      bool waves;
      //! Waves amplitude (m), direction (deg), frequency (rad/s).
      double waves_ampl, waves_dir, waves_freq;
      //! Waves amplitude, direction and frequency standard deviations.
      double stdev_waves_ampl, stdev_waves_dir, stdev_waves_freq;
      //! PRNG type.
      std::string prng_type;
      //! PRNG seed.
      int prng_seed;
    };

    struct Task: public Tasks::Task
    {
      //! Aboslute wind message.
      IMC::AbsoluteWind wind;
      //! Current message.
      IMC::SingleCurrentCell current;
      //! Waves message.
      IMC::WaveProfile waves;
      //! Pseudo-random generator.
      Random::Generator* m_prng;
      //! Timer.
      Time::Counter<float> m_timer;
      //! Task Arguments.
      Arguments m_args;

      Task(const std::string& name, Tasks::Context& ctx):
        Tasks::Task(name, ctx),
        m_prng(NULL)
      {
        param("Transmission period", m_args.transmission_period)
        .defaultValue("2")
        .units(Units::Second)
        .description("Period for information dispatching (s)");

        param("Measure wind", m_args.wind)
        .defaultValue("true")
        .description("Broadcast wind measurements if true");

        param("Wind direction", m_args.wind_dir)
        .units(Units::Degree)
        .description("Absolute NED wind direction (deg)");

        param("Standard Deviation - Wind direction", m_args.stdev_wind_dir)
        .units(Units::Degree)
        .defaultValue("0.75")
        .description("White noise added to wind direction");

        param("Wind speed", m_args.wind_vel)
        .units(Units::MeterPerSecond)
        .description("Absolute wind speed (m/s)");

        param("Standard Deviation - Wind speed", m_args.stdev_wind_vel)
        .units(Units::MeterPerSecond)
        .defaultValue("0.25")
        .description("White noise added to wind speed");

        param("Measure current", m_args.current)
        .defaultValue("true")
        .description("Broadcast current measurements if true");

        param("Current depth", m_args.curr_depth)
        .units(Units::Meter)
        .description("Absolute current depth (m)");

        param("Current direction", m_args.curr_dir)
        .units(Units::Degree)
        .description("Absolute NED current direction (deg)");

        param("Standard Deviation - Current direction", m_args.stdev_curr_dir)
        .units(Units::Degree)
        .defaultValue("0.15")
        .description("White noise added to current direction");

        param("Current speed", m_args.curr_vel)
        .units(Units::MeterPerSecond)
        .description("Absolute current speed (m/s)");

        param("Standard Deviation - Current speed", m_args.stdev_curr_vel)
        .units(Units::MeterPerSecond)
        .defaultValue("0.15")
        .description("White noise added to current speed");

        param("Measure waves", m_args.waves)
        .defaultValue("true")
        .description("Broadcast waves measurements if true");

        param("Waves amplitude", m_args.waves_ampl)
        .units(Units::Meter)
        .description("Wave amplitude (m)");

        param("Standard Deviation - Waves amplitude", m_args.stdev_waves_ampl)
        .units(Units::Meter)
        .defaultValue("0.3")
        .description("White noise added to waves amplitude");

        param("Waves direction", m_args.waves_dir)
        .units(Units::Degree)
        .description("Waves NED direction (deg)");

        param("Standard Deviation - Waves direction", m_args.stdev_waves_dir)
        .units(Units::Degree)
        .defaultValue("1")
        .description("White noise added to waves direction");

        param("Waves frequency", m_args.waves_freq)
        .units(Units::RadianPerSecond)
        .description("Waves frequency (rad/s)");

        param("Standard Deviation - Waves frequency", m_args.stdev_waves_freq)
        .units(Units::RadianPerSecond)
        .defaultValue("0.3")
        .description("White noise added to waves frequency");

        param("PRNG Type", m_args.prng_type)
        .defaultValue(Random::Factory::c_default);

        param("PRNG Seed", m_args.prng_seed)
        .defaultValue("-1");

        //bind<IMC::GpsFix>(this);
        //bind<IMC::SimulatedState>(this);
      }

      void
      onResourceAcquisition(void)
      {
        m_prng = Random::Factory::create(m_args.prng_type, m_args.prng_seed);
      }

      void
      onResourceRelease(void)
      {
        Memory::clear(m_prng);
      }

      void
      onUpdateParameters(void)
      {
        if(paramChanged(m_args.transmission_period))
          m_timer.setTop(m_args.transmission_period);
      }

      void
      onResourceInitialization(void)
      {
        m_timer.setTop(m_args.transmission_period);
        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_IDLE);
      }

      void
      broadcast(void)
      {
        //! Wind.
        if(m_args.wind)
        {
          wind.dir = m_args.wind_dir + (m_prng->gaussian() * m_args.stdev_wind_dir);
          wind.speed = m_args.wind_vel + (m_prng->gaussian() * m_args.stdev_wind_vel);
          dispatch(wind);
          debug("Simulated wind dir %.3f and speed %.3f", wind.dir, wind.speed);
        }
        //! Current.
        if(m_args.current)
        {
          current.depth = std::to_string(m_args.curr_depth);
          std::string dir_str = std::to_string(m_args.curr_dir + (m_prng->gaussian() * m_args.stdev_curr_dir));
          current.dir = dir_str;
          std::string vel_str = std::to_string(m_args.curr_vel + (m_prng->gaussian() * m_args.stdev_curr_vel));
          current.vel = vel_str;
          dispatch(current);
          debug("Simulated current depth %s, dir %s and speed %s", current.depth.c_str(), current.dir.c_str(), current.vel.c_str());
        }
        //! Waves.
        if(m_args.waves)
        {
          waves.ampl = m_args.waves_ampl + (m_prng->gaussian() * m_args.stdev_waves_ampl);
          waves.dir = m_args.waves_dir + (m_prng->gaussian() * m_args.stdev_waves_dir);
          waves.freq = m_args.waves_freq + (m_prng->gaussian() * m_args.stdev_waves_freq);
          dispatch(waves);
          debug("Simulated waves amplitude %.3f, dir %.3f and frequency %.3f", waves.ampl, waves.dir, waves.freq);
        }
      }

      void
      onMain(void)
      {
        while (!stopping())
        {
          waitForMessages(1.0);

          if(m_timer.overflow())
          {
            broadcast();
            m_timer.reset();
          }
        }
      }
    };
  }
}

DUNE_TASK
