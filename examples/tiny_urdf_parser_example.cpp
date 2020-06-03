// Copyright 2020 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <fstream>
#include <iostream>
#include <streambuf>
#include <string>

#include "tiny_double_utils.h"
#include "tiny_file_utils.h"
#include "tiny_urdf_parser.h"

int main(int argc, char *argv[]) {
  std::string file_name;
  TinyFileUtils::find_file("laikago/laikago_toes_zup.urdf", file_name);

  std::ifstream ifs(file_name);
  std::string urdf_string;

  if (!ifs.is_open()) {
    std::cout << "Error, cannot open file_name: " << file_name << std::endl;
    exit(-1);
  }

  urdf_string = std::string((std::istreambuf_iterator<char>(ifs)),
                            std::istreambuf_iterator<char>());

  StdLogger logger;

  TinyUrdfParser<double, DoubleUtils> parser;
  TinyUrdfStructures<double, DoubleUtils> urdf_structures;
  int flags = 0;
  parser.load_urdf_from_string(urdf_string, flags, logger, urdf_structures);
  printf("finished\n");
  return EXIT_SUCCESS;
}
