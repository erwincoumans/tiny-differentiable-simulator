/*
 * Copyright 2020 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MOTION_IMPORT_H
#define MOTION_IMPORT_H

#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <regex>

namespace tds {
enum LoopMode {
  LOOP_CLAMP = 0,  // stop at the last frame
  LOOP_WRAP        // loop at the last frame
};

struct Motion {
  std::vector<std::vector<double>> frames;
  LoopMode loop_mode{LOOP_WRAP};
  /**
   * Time length of a single frame in seconds.
   */
  double frame_duration{0};

  double total_duration() const { return frame_duration * frames.size(); }

 private:
  void get_blend_parameters(double time, int* idx_left, int* idx_right,
                            double* alpha) const {
    assert(!frames.empty());
    // improve numerical stability
    int n = static_cast<int>(
        std::floor(time / frame_duration + frame_duration / 4.));
    int num_frames = static_cast<int>(frames.size());
    *idx_left = n % num_frames;
    *idx_right = *idx_left + 1;
    if (*idx_right == num_frames) {
      if (loop_mode == LOOP_CLAMP) {
        *idx_right = *idx_left;
      } else {
        *idx_right = 0;
      }
    }
    //    *alpha = std::fmod(time, frame_duration) / frame_duration;
    // numerically stable:
    *alpha = (time - n * frame_duration) / frame_duration;
  }

 public:
  /**
   * Linear interpolation between frames to get the coordinates at an arbitrary
   * time.
   */
  std::vector<double> calculate_frame(double time) const {
    assert(!frames.empty());
    int idx_left;
    int idx_right;
    double alpha;
    get_blend_parameters(time, &idx_left, &idx_right, &alpha);
    std::vector<double> result;
    for (int i = 0; i < static_cast<int>(frames[0].size()); ++i) {
      result.emplace_back((1. - alpha) * frames[idx_left][i] +
                          alpha * frames[idx_right][i]);
    }
    // TODO: normalize rotation quaternion? (we do not use it right now anyway)
    return result;
  }

  static bool load_from_file(const std::string& filename, Motion* motion) {
    assert(bool(motion));
    std::ifstream input(filename);
    if (!input) {
      std::cerr << "Could not open motion file \"" << filename << "\".\n";
      return false;
    }
    std::cout << "Opening motion file \"" << filename << "\".\n";
    std::smatch match;
    std::regex loop_rx(".*\"LoopMode\":.*\"([A-Za-z]+)\".*");
    std::regex frame_duration_rx(R"(.*"FrameDuration":.*?([0-9\.]+).*)");
    std::regex frames_start_rx(R"(.*"Frames":.*)");
    std::regex frame_entry_rx(".*?([\\-0-9\\.]+).*?");
    for (std::string line; getline(input, line);) {
      if (std::regex_match(line, match, loop_rx) && match.size() == 2) {
        std::string loop_mode_str = match[1].str();
        std::cout << "LoopMode: " << loop_mode_str << "\n";
        if (loop_mode_str == "Wrap") {
          motion->loop_mode = LOOP_WRAP;
        } else {
          motion->loop_mode = LOOP_CLAMP;
        }
      } else if (std::regex_match(line, match, frame_duration_rx) &&
                 match.size() == 2) {
        std::string frame_duration_str = match[1].str();
        std::cout << "Duration: " << frame_duration_str << "\n";
        motion->frame_duration = std::stod(frame_duration_str);
      } else if (std::regex_match(line, match, frames_start_rx)) {
        // loop over the frames
        // XXX we assume each frame is written in a single line
        for (std::string frame_line; getline(input, frame_line);) {
          std::vector<double> frame;
          while (std::regex_search(frame_line, match, frame_entry_rx)) {
            frame.push_back(std::stod(match[1].str()));
            frame_line = match.suffix();
          }
          if (!frame.empty()) {
            motion->frames.push_back(frame);
          }
        }
      }
    }
    printf("Parsed %i frames.\n", static_cast<int>(motion->frames.size()));
    return true;
  }
};
}  // namespace tds

#endif  // MOTION_IMPORT_H
