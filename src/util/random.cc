// Copyright 2017 kvedder@umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
// Random number library.
//
//========================================================================
// This software is free: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License Version 3,
// as published by the Free Software Foundation.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// Version 3 in the file COPYING that came with this distribution.
// If not, see <http://www.gnu.org/licenses/>.
//========================================================================
#include "util/random.h"

using std::thread;

namespace minutebotrandom {

Random::Random()
    : thread_rng_seed(std::hash<std::thread::id>()(std::this_thread::get_id()) +
                      static_cast<int>(GetWallTime())) {}

Random::~Random() {}

int Random::RandomInt(size_t min, size_t max) {
  return (rand_r(&thread_rng_seed) % max) + min;
}

double Random::UniformRandom(double a, double b) {
  return (b - a) * UniformRandom() + a;
}

double Random::UniformRandom() {
  return rand_r(&thread_rng_seed) / static_cast<double>(RAND_MAX);
}

}  // namespace minutebotrandom
