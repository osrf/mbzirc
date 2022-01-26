/*
 * Copyright (C) 2019  Rhys Mainwaring
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/// \file Utilities.hh
/// \brief This file defines utilities for extracting parameters from SDF.

#ifndef IGNITION_GAZEBO_SYSTEMS_UTILITIES_HH_
#define IGNITION_GAZEBO_SYSTEMS_UTILITIES_HH_

#include <string>

#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>

#include <ignition/gazebo/config.hh>

#include <sdf/sdf.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  /// \brief A collection of static methods for common tasks.
  class Utilities
  {
    /// \brief Extract a named bool parameter from an SDF element.
    ///
    /// \param[in] _sdf         A reference to the SDF Element tree.
    /// \param[in] _paramName   The parameter name as it appears in SDF.
    /// \param[in] _defaultVal  A default value for the parameter.
    /// \return                 The parameter value
    ///                         (or default value if not found).
    public: static bool SdfParamBool(const sdf::Element& _sdf,
      const std::string &_paramName, const bool _defaultVal);

    /// \brief Extract a named size_t parameter from an SDF element.
    ///
    /// \param[in] _sdf         A reference to the SDF Element tree.
    /// \param[in] _paramName   The parameter name as it appears in SDF.
    /// \param[in] _defaultVal  A default value for the parameter.
    /// \return                 The parameter value
    ///                         (or default value if not found).

    public: static size_t SdfParamSizeT(const sdf::Element& _sdf,
      const std::string &_paramName, const size_t _defaultVal);

    /// \brief Extract a named double parameter from an SDF element.
    ///
    /// \param[in] _sdf         A reference to the SDF Element tree.
    /// \param[in] _paramName   The parameter name as it appears in SDF.
    /// \param[in] _defaultVal  A default value for the parameter.
    /// \return                 The parameter value
    ///                         (or default value if not found).

    public: static double SdfParamDouble(const sdf::Element& _sdf,
      const std::string &_paramName, const double _defaultVal);

    /// \brief Extract a named string parameter from an SDF element.
    ///
    /// \param[in] _sdf         A reference to the SDF Element tree.
    /// \param[in] _paramName   The parameter name as it appears in SDF.
    /// \param[in] _defaultVal  A default value for the parameter.
    /// \return                 The parameter value
    ///                         (or default value if not found).

    public: static std::string SdfParamString(const sdf::Element& _sdf,
      const std::string &_paramName, const std::string &_defaultVal);

    /// \brief Extract a named Vector2 parameter from an SDF element.
    ///
    /// \param[in] _sdf         A reference to the SDF Element tree.
    /// \param[in] _paramName   The parameter name as it appears in SDF.
    /// \param[in] _defaultVal  A default value for the parameter.
    /// \return                 The parameter value
    ///                         (or default value if not found).

    public: static ignition::math::Vector2d SdfParamVector2(
      const sdf::Element& _sdf,
      const std::string &_paramName,
      const ignition::math::Vector2d _defaultVal);

    /// \brief Extract a named Vector3 parameter from an SDF element.
    ///
    /// \param[in] _sdf         A reference to the SDF Element tree.
    /// \param[in] _paramName   The parameter name as it appears in SDF.
    /// \param[in] _defaultVal  A default value for the parameter.
    /// \return                 The parameter value
    ///                         (or default value if not found).

    public: static ignition::math::Vector3d SdfParamVector3(
      const sdf::Element& _sdf,
      const std::string &_paramName,
      const ignition::math::Vector3d _defaultVal);
  };
  }
}
}
}

#endif
