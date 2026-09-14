// Copyright 2021 Tier IV, Inc. All rights reserved.
// Copyright 2026 Alistair English, Emerson Knapp
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef THIN_OCCUPANCY_GRID_MAP_OUTLIER_FILTER__VISIBILITY_CONTROL_HPP_
#define THIN_OCCUPANCY_GRID_MAP_OUTLIER_FILTER__VISIBILITY_CONTROL_HPP_

// Symbol visibility macros. Only the handful of symbols that form the public
// link-time API are annotated with THIN_OGM_OUTLIER_FILTER_PUBLIC; everything
// else stays hidden. See https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define THIN_OGM_OUTLIER_FILTER_EXPORT __attribute__((dllexport))
    #define THIN_OGM_OUTLIER_FILTER_IMPORT __attribute__((dllimport))
  #else
    #define THIN_OGM_OUTLIER_FILTER_EXPORT __declspec(dllexport)
    #define THIN_OGM_OUTLIER_FILTER_IMPORT __declspec(dllimport)
  #endif
  #ifdef THIN_OGM_OUTLIER_FILTER_BUILDING_LIBRARY
    #define THIN_OGM_OUTLIER_FILTER_PUBLIC THIN_OGM_OUTLIER_FILTER_EXPORT
  #else
    #define THIN_OGM_OUTLIER_FILTER_PUBLIC THIN_OGM_OUTLIER_FILTER_IMPORT
  #endif
#else
  #define THIN_OGM_OUTLIER_FILTER_EXPORT __attribute__((visibility("default")))
  #define THIN_OGM_OUTLIER_FILTER_IMPORT
  #define THIN_OGM_OUTLIER_FILTER_PUBLIC __attribute__((visibility("default")))
#endif

#endif  // THIN_OCCUPANCY_GRID_MAP_OUTLIER_FILTER__VISIBILITY_CONTROL_HPP_
